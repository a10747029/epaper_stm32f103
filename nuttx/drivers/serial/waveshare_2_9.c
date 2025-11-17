/****************************************************************************
 * drivers/epaper/waveshare_2_9.c
 *
 * Waveshare 2.9" 黑白电子纸(V2)驱动 (基于你提供的命令表 Images 1~5)
 *
 * 支持特性:
 *  - 分辨率: 128(W) x 296(H), 1bpp
 *  - SPI 接口 (NuttX SPI 框架 + SPI_CMDDATA 回调控制 D/C)
 *  - 设备节点: /dev/epaper
 *  - 完整初始化序列: 复位 -> 上电/Booster -> 基本面板配置 -> 写 LUT -> 时序参数 -> 清屏刷新
 *  - IOCTL:
 *      EPAPER_IOC_GETINFO     : 获取 width/height/bpp
 *      EPAPER_IOC_CLEAR       : 白色填充并整屏刷新
 *      EPAPER_IOC_REFRESH     : 根据当前 RAM 内容刷新
 *      EPAPER_IOC_SLEEP       : 深度休眠
 *      EPAPER_IOC_BLIT        : 局部写入 (结构 epaper_blit)
 *      EPAPER_IOC_SETLUT      : 设置并写入 30 字节 LUT (不含 VSH/VSL & dummy bit)
 *      EPAPER_IOC_SETBORDER   : 写 0x3C (Border Waveform Control)
 *      EPAPER_IOC_SETUPDATE1  : 写 0x21 (Display Update Control 1)
 *      EPAPER_IOC_SETUPDATE2  : 写 0x22 (Display Update Control 2)
 *      EPAPER_IOC_SETVCOM     : 写 0x2C (VCOM)
 *      EPAPER_IOC_SETDUMMY    : 写 0x3A (Dummy line period)
 *      EPAPER_IOC_SETGATEWIDTH: 写 0x3B (Gate line width)
 *      EPAPER_IOC_SETDATAENTRY: 写 0x11 (Data Entry mode)
 *      EPAPER_IOC_TEMPWRITE   : 写 0x1A (温度传感寄存器, 2 字节: MSB | LSB)
 *
 * 使用:
 *   1. open("/dev/epaper", O_RDWR)
 *   2. 按需调用 BLIT 写局部图形 (建议 x,w 8 像素对齐提升效率)
 *   3. 如需改变刷新选项, 使用 SETUPDATE1/SETUPDATE2 设置参数
 *   4. 调用 REFRESH 进行显示
 *
 * 注意:
 *  - Busy 高电平表示忙, 等待下降为就绪
 *  - BLIT 对非 8 对齐的 x/w 做内部打包, 可能影响相邻像素 (建议对齐)
 *  - 当前实现只做整屏刷新序列, 未实现真正节能的局部差分刷新 LUT
 *  - LUT 默认给出一个占位（需根据实际器件官方 LUT 微调）
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/epaper/waveshare_2_9.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* 外部 GPIO 接口(由 arch 层实现) */
extern bool stm32_gpioread(uint32_t cfgset);
extern void stm32_gpiowrite(uint32_t cfgset, bool value);

#ifndef CONFIG_SPI
#  error "EPD driver requires CONFIG_SPI"
#endif
#ifndef CONFIG_SPI_CMDDATA
#  error "EPD driver requires CONFIG_SPI_CMDDATA"
#endif

/****************************************************************************
 * 固定参数
 ****************************************************************************/
#ifndef EPD_WIDTH
#  define EPD_WIDTH   128u
#endif
#ifndef EPD_HEIGHT
#  define EPD_HEIGHT  296u
#endif
#define EPD_BPP            1u
#define EPD_FB_SIZE        ((EPD_WIDTH * EPD_HEIGHT)/8u)

/****************************************************************************
 * IOCTL 基础 + 扩展
 ****************************************************************************/
#ifndef EPAPER_IOC_BASE
#  define EPAPER_IOC_BASE          0xE0
#endif
#ifndef EPAPER_IOC_REFRESH
#  define EPAPER_IOC_REFRESH       (EPAPER_IOC_BASE + 1)
#endif
#ifndef EPAPER_IOC_SLEEP
#  define EPAPER_IOC_SLEEP         (EPAPER_IOC_BASE + 2)
#endif
#ifndef EPAPER_IOC_CLEAR
#  define EPAPER_IOC_CLEAR         (EPAPER_IOC_BASE + 3)
#endif
#ifndef EPAPER_IOC_GETINFO
#  define EPAPER_IOC_GETINFO       (EPAPER_IOC_BASE + 10)
#endif
#ifndef EPAPER_IOC_BLIT
#  define EPAPER_IOC_BLIT          (EPAPER_IOC_BASE + 11)
#endif

#define EPAPER_IOC_SETLUT          (EPAPER_IOC_BASE + 12) /* arg=30字节 */
#define EPAPER_IOC_SETBORDER       (EPAPER_IOC_BASE + 13) /* arg=1字节(写0x3C) */
#define EPAPER_IOC_SETUPDATE1      (EPAPER_IOC_BASE + 14) /* arg=1字节(写0x21) */
#define EPAPER_IOC_SETUPDATE2      (EPAPER_IOC_BASE + 15) /* arg=1字节(写0x22) */
#define EPAPER_IOC_SETVCOM         (EPAPER_IOC_BASE + 16) /* arg=1字节(写0x2C) */
#define EPAPER_IOC_SETDUMMY        (EPAPER_IOC_BASE + 17) /* arg=1字节(写0x3A) */
#define EPAPER_IOC_SETGATEWIDTH    (EPAPER_IOC_BASE + 18) /* arg=1字节(写0x3B) */
#define EPAPER_IOC_SETDATAENTRY    (EPAPER_IOC_BASE + 19) /* arg=1字节(写0x11) */
#define EPAPER_IOC_TEMPWRITE       (EPAPER_IOC_BASE + 20) /* arg=2字节(写0x1A MSB|LSB) */

struct epaper_info
{
  uint16_t width;
  uint16_t height;
  uint8_t  bpp;
  uint8_t  reserved;
};

struct epaper_blit
{
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  FAR const void *addr; /* 指向 w*h/8 字节 1bpp 数据, MSB-first */
};

/****************************************************************************
 * 命令宏 (与 datasheet 对应)
 ****************************************************************************/
#define CMD_DRIVER_OUTPUT_CONTROL      0x01  /* (也可能与 Power Setting 并列) */
#define CMD_BOOSTER_SOFT_START         0x0C
#define CMD_DEEP_SLEEP_MODE            0x10
#define CMD_DATA_ENTRY_MODE_SETTING    0x11
#define CMD_SWRESET                    0x12
#define CMD_WRITE_RAM                  0x24
#define CMD_WRITE_VCOM_REGISTER        0x2C
#define CMD_WRITE_LUT_REGISTER         0x32
#define CMD_SET_DUMMY_LINE_PERIOD      0x3A
#define CMD_SET_GATE_LINE_WIDTH        0x3B
#define CMD_BORDER_WAVEFORM_CONTROL    0x3C
#define CMD_TEMP_SENSOR_CONTROL        0x1A
#define CMD_MASTER_ACTIVATION          0x20
#define CMD_DISPLAY_UPDATE_CONTROL_1   0x21
#define CMD_DISPLAY_UPDATE_CONTROL_2   0x22
#define CMD_SET_RAM_X_ADDRESS          0x44
#define CMD_SET_RAM_Y_ADDRESS          0x45
#define CMD_SET_RAM_X_COUNTER          0x4E
#define CMD_SET_RAM_Y_COUNTER          0x4F
#define CMD_NOP                        0xFF

/* 部分控制器扩展命令 */
#define CMD_POWER_SETTING              0x01 /* 同上(具体取决于文档版本) */
#define CMD_POWER_ON                   0x04
#define CMD_VCOM_AND_DATA_INTERVAL     0x50
#define CMD_TCON_RESOLUTION            0x61
#define CMD_VCM_DC_SETTING             0x82
#define CMD_PLL_CONTROL                0x30
#define CMD_DISPLAY_REFRESH            0x12 /* 某些版本: 0x12 为刷新触发 */

/****************************************************************************
 * 私有结构
 ****************************************************************************/
struct ws29_dev_s
{
  FAR struct spi_dev_s *spi;
  uint32_t gpio_busy;
  uint32_t gpio_rst;

  sem_t    exclsem;
  bool     powered;

  uint16_t width;
  uint16_t height;
  size_t   fbsize;
  FAR uint8_t *fb;          /* 可选整屏缓冲 (用于 write/clear) */

  /* 缓存设置参数 */
  uint8_t update_ctrl_1; /* 最近一次写 0x21 */
  uint8_t update_ctrl_2; /* 最近一次写 0x22 */
  uint8_t border_ctrl;   /* 0x3C */
  uint8_t data_entry;    /* 0x11 */
  uint8_t dummy_line;    /* 0x3A */
  uint8_t gate_line_w;   /* 0x3B */
  uint8_t vcom;          /* 0x2C */
  uint8_t lut[30];       /* 当前 30 字节 LUT */
};

/****************************************************************************
 * 前向声明
 ****************************************************************************/
static int     ws29_open(FAR struct file *filep);
static int     ws29_close(FAR struct file *filep);
static ssize_t ws29_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t ws29_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     ws29_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int     sem_wait_unintr(sem_t *sem);
static void    epd_cmd(FAR struct ws29_dev_s *priv, uint8_t cmd);
static void    epd_data(FAR struct ws29_dev_s *priv, FAR const uint8_t *data, size_t len);
static int     epd_wait_busy(FAR struct ws29_dev_s *priv, unsigned int timeout_ms);
static void    epd_reset(FAR struct ws29_dev_s *priv);
static int     epd_init_hw(FAR struct ws29_dev_s *priv);

static void    epd_set_window(FAR struct ws29_dev_s *priv,
                              uint16_t xs, uint16_t xe,
                              uint16_t ys, uint16_t ye);
static void    epd_set_cursor(FAR struct ws29_dev_s *priv,
                              uint16_t x, uint16_t y);

static int     epd_full_write(FAR struct ws29_dev_s *priv,
                              FAR const uint8_t *fb, size_t len);
static int     epd_blit(FAR struct ws29_dev_s *priv,
                        uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        FAR const uint8_t *bits);

static int     epd_clear(FAR struct ws29_dev_s *priv);
static int     epd_refresh(FAR struct ws29_dev_s *priv);
static int     epd_sleep(FAR struct ws29_dev_s *priv);

/****************************************************************************
 * File operations
 ****************************************************************************/
static const struct file_operations g_epaper_fops =
{
  ws29_open,
  ws29_close,
  ws29_read,
  ws29_write,
  NULL,
  ws29_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * 基础 SPI/同步 辅助函数
 ****************************************************************************/
static int sem_wait_unintr(sem_t *sem)
{
  int ret;
  do
    {
      ret = sem_wait(sem);
    }
  while (ret < 0 && errno == EINTR);
  return ret;
}

static void epd_cmd(FAR struct ws29_dev_s *priv, uint8_t cmd)
{
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  (void)SPI_SEND(priv->spi, cmd);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static void epd_data(FAR struct ws29_dev_s *priv, FAR const uint8_t *data, size_t len)
{
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  for (size_t i=0; i<len; i++)
    {
      (void)SPI_SEND(priv->spi, data[i]);
    }
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static int epd_wait_busy(FAR struct ws29_dev_s *priv, unsigned int timeout_ms)
{
  unsigned int waited = 0;
  while (stm32_gpioread(priv->gpio_busy))
    {
      if (waited >= timeout_ms)
        {
          return -ETIMEDOUT;
        }
      up_mdelay(1);
      waited++;
    }
  return OK;
}

static void epd_reset(FAR struct ws29_dev_s *priv)
{
  stm32_gpiowrite(priv->gpio_rst, false);
  up_mdelay(10);
  stm32_gpiowrite(priv->gpio_rst, true);
  up_mdelay(10);
}

/****************************************************************************
 * RAM 窗口与指针
 ****************************************************************************/
static void epd_set_window(FAR struct ws29_dev_s *priv,
                           uint16_t xs, uint16_t xe,
                           uint16_t ys, uint16_t ye)
{
  /* X byte addressed */
  epd_cmd(priv, CMD_SET_RAM_X_ADDRESS);
  uint8_t xdat[2] = { (uint8_t)(xs/8), (uint8_t)(xe/8) };
  epd_data(priv, xdat, 2);

  epd_cmd(priv, CMD_SET_RAM_Y_ADDRESS);
  uint8_t ydat[4] = {
    (uint8_t)(ys & 0xFF), (uint8_t)((ys>>8)&0xFF),
    (uint8_t)(ye & 0xFF), (uint8_t)((ye>>8)&0xFF)
  };
  epd_data(priv, ydat, 4);
}

static void epd_set_cursor(FAR struct ws29_dev_s *priv,
                           uint16_t x, uint16_t y)
{
  epd_cmd(priv, CMD_SET_RAM_X_COUNTER);
  uint8_t xbyte = (uint8_t)(x/8);
  epd_data(priv, &xbyte, 1);

  epd_cmd(priv, CMD_SET_RAM_Y_COUNTER);
  uint8_t ydat[2] = { (uint8_t)(y & 0xFF), (uint8_t)((y>>8)&0xFF) };
  epd_data(priv, ydat, 2);
}

/****************************************************************************
 * 全屏写入 / 清屏 / 刷新 / 休眠
 ****************************************************************************/
static int epd_full_write(FAR struct ws29_dev_s *priv,
                          FAR const uint8_t *fb, size_t len)
{
  if (len < priv->fbsize)
    {
      return -EINVAL;
    }
  epd_set_window(priv, 0, priv->width - 1, 0, priv->height - 1);
  epd_set_cursor(priv, 0, 0);

  epd_cmd(priv, CMD_WRITE_RAM);
  /* 分块发送 */
  size_t remain = priv->fbsize;
  while (remain > 0)
    {
      size_t chunk = remain > 128 ? 128 : remain;
      epd_data(priv, fb, chunk);
      fb     += chunk;
      remain -= chunk;
    }
  return OK;
}

static int epd_clear(FAR struct ws29_dev_s *priv)
{
  epd_set_window(priv, 0, priv->width - 1, 0, priv->height - 1);
  epd_set_cursor(priv, 0, 0);
  epd_cmd(priv, CMD_WRITE_RAM);

  /* 白色填充 (1=白) */
  uint8_t block[64];
  memset(block, 0xFF, sizeof(block));

  size_t remain = priv->fbsize;
  while (remain > 0)
    {
      size_t n = remain > sizeof(block) ? sizeof(block) : remain;
      epd_data(priv, block, n);
      remain -= n;
    }
  return OK;
}

static int epd_refresh(FAR struct ws29_dev_s *priv)
{
  /* 基于命令表: 先写 Display Update Control 1/2 (如已设置可省略) 再 Master Activation */
  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_1);
  epd_data(priv, &priv->update_ctrl_1, 1);

  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_2);
  epd_data(priv, &priv->update_ctrl_2, 1);

  epd_cmd(priv, CMD_MASTER_ACTIVATION);
  return epd_wait_busy(priv, 15000);
}

static int epd_sleep(FAR struct ws29_dev_s *priv)
{
  epd_cmd(priv, CMD_DEEP_SLEEP_MODE);
  uint8_t v = 0x01; /* 进入睡眠 */
  epd_data(priv, &v, 1);
  priv->powered = false;
  return OK;
}

/****************************************************************************
 * 局部写入 (BLIT)
 * bits: w*h/8 字节的 MSB-first 1bpp 图像
 ****************************************************************************/
static void pack_row(uint8_t *dst, uint16_t dst_bytes,
                     const uint8_t *src, uint16_t w_bits,
                     uint8_t shift_left)
{
  memset(dst, 0xFF, dst_bytes); /* 默认全白 */
  for (uint16_t sb=0; sb<w_bits; sb++)
    {
      uint16_t sbyte = sb / 8u;
      uint8_t  smask = (uint8_t)(0x80u >> (sb & 7u));
      bool black = (src[sbyte] & smask) == 0; /* 0=黑 */

      uint16_t dbit = sb + shift_left;
      uint16_t dbyte = dbit / 8u;
      uint8_t  dmask = (uint8_t)(0x80u >> (dbit & 7u));
      if (dbyte >= dst_bytes) break;

      if (black)
        {
          dst[dbyte] &= (uint8_t)(~dmask);
        }
      else
        {
          dst[dbyte] |= dmask;
        }
    }
}

static int epd_blit(FAR struct ws29_dev_s *priv,
                    uint16_t x, uint16_t y,
                    uint16_t w, uint16_t h,
                    FAR const uint8_t *bits)
{
  if (!bits || w==0 || h==0) return -EINVAL;
  if (x >= priv->width || y >= priv->height) return -EINVAL;
  if ((uint32_t)x + w > priv->width || (uint32_t)y + h > priv->height) return -EINVAL;

  uint16_t x_aligned = (uint16_t)(x & ~7u);
  uint8_t  shift     = (uint8_t)(x - x_aligned);
  uint16_t dst_bytes = (uint16_t)((shift + w + 7u)/8u);
  uint16_t src_bpr   = (uint16_t)((w + 7u)/8u);

  /* 设置窗口覆盖该区域 (按字节对齐扩张) */
  epd_set_window(priv,
                 x_aligned,
                 (uint16_t)(x_aligned + dst_bytes*8u - 1u),
                 y,
                 (uint16_t)(y + h - 1u));

  uint8_t *rowbuf;
  bool dyn = false;
  if (dst_bytes <= 64)
    {
      static uint8_t static_row[64];
      rowbuf = static_row;
    }
  else
    {
      rowbuf = kmm_malloc(dst_bytes);
      if (!rowbuf) return -ENOMEM;
      dyn = true;
    }

  for (uint16_t row=0; row<h; row++)
    {
      const uint8_t *src_row = bits + row*src_bpr;
      pack_row(rowbuf, dst_bytes, src_row, w, shift);

      epd_set_cursor(priv, x_aligned, (uint16_t)(y + row));
      epd_cmd(priv, CMD_WRITE_RAM);
      epd_data(priv, rowbuf, dst_bytes);
    }

  if (dyn) kmm_free(rowbuf);
  return OK;
}

/****************************************************************************
 * 初始化序列
 * 依据常见 Waveshare 2.9" V2 的参考流程 (你的命令表 + 常用寄存器)
 ****************************************************************************/
static int epd_init_hw(FAR struct ws29_dev_s *priv)
{
  epd_reset(priv);

  /* Software reset */
  epd_cmd(priv, CMD_SWRESET);
  if (epd_wait_busy(priv, 5000) < 0) return -ETIMEDOUT;

  /* Driver Output Control (0x01):
   * 高度-1, 然后 GD/SM/TB(按 POR 通常 0x00)
   * 高度=296 => 295 = 0x0127 (低字节 0x27, 高字节 0x01)
   * 部分文档需要 3 字节: (Height-1 LSB), (Height-1 MSB), 0x00
   */
  epd_cmd(priv, CMD_DRIVER_OUTPUT_CONTROL);
  {
    uint8_t doc[3] = { 0x27, 0x01, 0x00 };
    epd_data(priv, doc, 3);
  }

  /* Booster Soft Start (0x0C) 示例值: 0xD7 0xD6 0x9D (不同文档略有差异) */
  epd_cmd(priv, CMD_BOOSTER_SOFT_START);
  {
    uint8_t bss[3] = { 0xD7, 0xD6, 0x9D };
    epd_data(priv, bss, 3);
  }

  /* VCOM 设置 (0x2C) 示例: 0x7F 或 datasheet 推荐值 */
  priv->vcom = 0x7F;
  epd_cmd(priv, CMD_WRITE_VCOM_REGISTER);
  epd_data(priv, &priv->vcom, 1);

  /* Dummy line period (0x3A) POR: 0x1A */
  priv->dummy_line = 0x1A;
  epd_cmd(priv, CMD_SET_DUMMY_LINE_PERIOD);
  epd_data(priv, &priv->dummy_line, 1);

  /* Gate line width (0x3B) POR: 0x08 */
  priv->gate_line_w = 0x08;
  epd_cmd(priv, CMD_SET_GATE_LINE_WIDTH);
  epd_data(priv, &priv->gate_line_w, 1);

  /* Data entry mode (0x11) 常用 0x03: X++, Y++ */
  priv->data_entry = 0x03;
  epd_cmd(priv, CMD_DATA_ENTRY_MODE_SETTING);
  epd_data(priv, &priv->data_entry, 1);

  /* Border waveform (0x3C) POR: 0x01 或 0x05 视实现
     例如使用 0x05: GS Transition + 白边框; 这里采用 0x05 */
  priv->border_ctrl = 0x05;
  epd_cmd(priv, CMD_BORDER_WAVEFORM_CONTROL);
  epd_data(priv, &priv->border_ctrl, 1);

  /* Display Update Control 1 (0x21):
     A7=OLD RAM bypass; A4=值用于 bypass; A1:A0=初始源选择
     常用整屏刷新: 0x00 或 0xF7(后续结合 0x22)
     这里先设 0x00 */
  priv->update_ctrl_1 = 0x00;
  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_1);
  epd_data(priv, &priv->update_ctrl_1, 1);

  /* Display Update Control 2 (0x22):
     位含义：Enable clock, Enable CP, Load LUT, Initial pattern, etc.
     常用整屏刷新一次: 0xF7
  */
  priv->update_ctrl_2 = 0xF7;
  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_2);
  epd_data(priv, &priv->update_ctrl_2, 1);

  /* LUT (0x32) 写入 30 字节: 占位默认表(示例) */
  static const uint8_t default_lut[30] =
  {
    /* 真实值需参考官方 LUT 表，不同批次可能不同，这里仅给演示数据 */
    0x32,0x08,0x08,0x00,0x08,0x08,
    0x02,0x08,0x08,0x00,0x08,0x08,
    0x02,0x08,0x08,0x00,0x08,0x08,
    0x02,0x08,0x08,0x00,0x08,0x08,
    0x02,0x08,0x08,0x00,0x08,0x08
  };
  memcpy(priv->lut, default_lut, 30);
  epd_cmd(priv, CMD_WRITE_LUT_REGISTER);
  epd_data(priv, priv->lut, 30);

  /* 设置整屏窗口 */
  epd_set_window(priv, 0, priv->width - 1, 0, priv->height - 1);
  epd_set_cursor(priv, 0, 0);

  /* 清屏并刷新一次 */
  (void)epd_clear(priv);
  (void)epd_refresh(priv);

  priv->powered = true;
  return OK;
}

/****************************************************************************
 * File ops
 ****************************************************************************/
static int ws29_open(FAR struct file *filep)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;
  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }
  sem_post(&priv->exclsem);
  return OK;
}

static int ws29_close(FAR struct file *filep)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;
  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }
  sem_post(&priv->exclsem);
  return OK;
}

static ssize_t ws29_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* 本驱动暂不提供 frame RAM 读取功能，返回 0 */
  (void)filep; (void)buffer; (void)buflen;
  return 0;
}

static ssize_t ws29_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;
  if (!buffer || buflen < priv->fbsize)
    {
      return -EINVAL;
    }

  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }

  int ret = epd_full_write(priv, (FAR const uint8_t*)buffer, buflen);
  if (ret == OK)
    {
      ret = epd_refresh(priv);
    }

  sem_post(&priv->exclsem);
  return ret == OK ? (ssize_t)priv->fbsize : (ssize_t)ret;
}

static int ws29_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;
  int ret = OK;

  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }

  switch (cmd)
    {
      case EPAPER_IOC_GETINFO:
        {
          FAR struct epaper_info *info = (FAR struct epaper_info*)(uintptr_t)arg;
          if (!info)
            {
              ret = -EINVAL;
              break;
            }
            info->width  = priv->width;
            info->height = priv->height;
            info->bpp    = EPD_BPP;
        }
        break;

      case EPAPER_IOC_CLEAR:
        ret = epd_clear(priv);
        if (ret == OK) ret = epd_refresh(priv);
        break;

      case EPAPER_IOC_REFRESH:
        ret = epd_refresh(priv);
        break;

      case EPAPER_IOC_SLEEP:
        ret = epd_sleep(priv);
        break;

      case EPAPER_IOC_BLIT:
        {
          FAR struct epaper_blit *bl = (FAR struct epaper_blit*)(uintptr_t)arg;
          if (!bl || !bl->addr)
            {
              ret = -EINVAL;
              break;
            }
          ret = epd_blit(priv, bl->x, bl->y, bl->w, bl->h,
                         (FAR const uint8_t*)bl->addr);
        }
        break;

      case EPAPER_IOC_SETLUT:
        {
          FAR const uint8_t *lut = (FAR const uint8_t*)(uintptr_t)arg;
          if (!lut)
            {
              ret = -EINVAL;
              break;
            }
          memcpy(priv->lut, lut, 30);
          epd_cmd(priv, CMD_WRITE_LUT_REGISTER);
          epd_data(priv, priv->lut, 30);
        }
        break;

      case EPAPER_IOC_SETBORDER:
        {
          uint8_t v = (uint8_t)arg;
          priv->border_ctrl = v;
          epd_cmd(priv, CMD_BORDER_WAVEFORM_CONTROL);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETUPDATE1:
        {
          uint8_t v = (uint8_t)arg;
          priv->update_ctrl_1 = v;
          epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_1);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETUPDATE2:
        {
          uint8_t v = (uint8_t)arg;
          priv->update_ctrl_2 = v;
          epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_2);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETVCOM:
        {
          uint8_t v = (uint8_t)arg;
          priv->vcom = v;
          epd_cmd(priv, CMD_WRITE_VCOM_REGISTER);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETDUMMY:
        {
          uint8_t v = (uint8_t)arg;
          priv->dummy_line = v;
          epd_cmd(priv, CMD_SET_DUMMY_LINE_PERIOD);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETGATEWIDTH:
        {
          uint8_t v = (uint8_t)arg;
          priv->gate_line_w = v;
          epd_cmd(priv, CMD_SET_GATE_LINE_WIDTH);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETDATAENTRY:
        {
          uint8_t v = (uint8_t)arg;
          priv->data_entry = v;
          epd_cmd(priv, CMD_DATA_ENTRY_MODE_SETTING);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_TEMPWRITE:
        {
          FAR const uint8_t *tp = (FAR const uint8_t*)(uintptr_t)arg;
          /* 需要传入两个字节: A( MSB ), B( LSB ) */
          if (!tp)
            {
              ret = -EINVAL;
              break;
            }
          epd_cmd(priv, CMD_TEMP_SENSOR_CONTROL);
          epd_data(priv, tp, 2);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * 公共初始化入口
 ****************************************************************************/
int ws_epaper_initialize(FAR struct spi_dev_s *spi,
                         uint32_t gpio_busy,
                         uint32_t gpio_rst)
{
  FAR struct ws29_dev_s *priv;
  int ret;

  if (!spi)
    {
      return -ENODEV;
    }

  priv = (FAR struct ws29_dev_s*)kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      return -ENOMEM;
    }

  priv->spi       = spi;
  priv->gpio_busy = gpio_busy;
  priv->gpio_rst  = gpio_rst;
  priv->width     = EPD_WIDTH;
  priv->height    = EPD_HEIGHT;
  priv->fbsize    = EPD_FB_SIZE;

  /* 可选整屏缓冲 (当前仅 write() 用, 可不分配) */
  priv->fb = NULL; /* 若需要: priv->fb = kmm_malloc(priv->fbsize); */

  sem_init(&priv->exclsem, 0, 1);

  /* SPI 参数 */
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, 4000000);

  ret = epd_init_hw(priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "epaper: init hw failed: %d\n", ret);
      sem_destroy(&priv->exclsem);
      if (priv->fb) kmm_free(priv->fb);
      kmm_free(priv);
      return ret;
    }

  ret = register_driver("/dev/epaper", &g_epaper_fops, 0644, priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "epaper: register_driver failed: %d\n", ret);
      sem_destroy(&priv->exclsem);
      if (priv->fb) kmm_free(priv->fb);
      kmm_free(priv);
      return ret;
    }

  syslog(LOG_INFO,
         "epaper: /dev/epaper registered (%ux%u, fbsize=%u)\n",
         priv->width, priv->height, (unsigned)priv->fbsize);

  return OK;
}
