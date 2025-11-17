/****************************************************************************
 * drivers/epaper/waveshare_2_9.c
 *
 * Waveshare 2.9" 黑白电子纸(V2)驱动
 *
 * 参考官方示例 EPD_2in9.c (V3.0, 2019-06-12) 实现初始化逻辑：
 *  - 4 线 SPI：D/C 由 SPI_CMDDATA 控制 (NuttX board 层负责实际 GPIO)
 *  - RESET & BUSY 由 stm32_gpiowrite/stm32_gpioread 控制
 *  - 初始化时：配置寄存器 → 写官方 full LUT → 设置显示更新控制为强全刷 (0xF7)
 *  - 初始化阶段进行两轮“全白写入 + 全刷”，尽量清除上电残影
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
#  error "EPD driver requires CONFIG_SPI_CMDDATA (4-wire SPI with D/C GPIO)"
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
 * 命令宏 (与 datasheet 对应 / 对齐 Waveshare 代码)
 ****************************************************************************/
#define CMD_DRIVER_OUTPUT_CONTROL      0x01
#define CMD_BOOSTER_SOFT_START         0x0C
#define CMD_DEEP_SLEEP_MODE            0x10
#define CMD_DATA_ENTRY_MODE_SETTING    0x11
#define CMD_SWRESET                    0x12
#define CMD_TEMP_SENSOR_CONTROL        0x1A
#define CMD_MASTER_ACTIVATION          0x20
#define CMD_DISPLAY_UPDATE_CONTROL_1   0x21
#define CMD_DISPLAY_UPDATE_CONTROL_2   0x22
#define CMD_WRITE_RAM                  0x24
#define CMD_WRITE_VCOM_REGISTER        0x2C
#define CMD_WRITE_LUT_REGISTER         0x32
#define CMD_SET_DUMMY_LINE_PERIOD      0x3A
#define CMD_SET_GATE_LINE_WIDTH        0x3B
#define CMD_BORDER_WAVEFORM_CONTROL    0x3C
#define CMD_SET_RAM_X_ADDRESS          0x44
#define CMD_SET_RAM_Y_ADDRESS          0x45
#define CMD_SET_RAM_X_COUNTER          0x4E
#define CMD_SET_RAM_Y_COUNTER          0x4F
#define CMD_NOP                        0xFF

/****************************************************************************
 * 官方 Waveshare EPD_2IN9 LUT (full update)
 * 来自 EPD_2in9.c:
 * const unsigned char EPD_2IN9_lut_full_update[] = {
 *   0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00,
 *   0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 *   0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 *   0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00,
 *   0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * };
 ****************************************************************************/
static const uint8_t g_ws29_lut_full[30] =
{
  0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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
  ws29_open,        /* open  */
  ws29_close,       /* close */
  ws29_read,        /* read  */
  ws29_write,       /* write */
  NULL,             /* seek  */
  ws29_ioctl        /* ioctl */
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
  /* 4 线 SPI: D/C 由 SPI_CMDDATA 控制 (true=command) */
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  (void)SPI_SEND(priv->spi, cmd);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static void epd_data(FAR struct ws29_dev_s *priv, FAR const uint8_t *data, size_t len)
{
  /* 4 线 SPI: D/C 由 SPI_CMDDATA 控制 (false=data) */
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  for (size_t i = 0; i < len; i++)
    {
      (void)SPI_SEND(priv->spi, data[i]);
    }
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static int epd_wait_busy(FAR struct ws29_dev_s *priv, unsigned int timeout_ms)
{
  unsigned int waited = 0;

  /* Waveshare 注释：LOW: idle, HIGH: busy */
  while (stm32_gpioread(priv->gpio_busy))
    {
      if (waited >= timeout_ms)
        {
          syslog(LOG_ERR, "epaper: BUSY timeout after %u ms (busy pin still HIGH)\n",
                 waited);
          return -ETIMEDOUT;
        }

      up_mdelay(10);
      waited += 10;
    }

  if (waited > 0)
    {
      syslog(LOG_DEBUG, "epaper: BUSY cleared after %u ms\n", waited);
    }

  return OK;
}

static void epd_reset(FAR struct ws29_dev_s *priv)
{
  syslog(LOG_INFO, "epaper: hardware reset\n");

  /* 参考 EPD_2IN9_Reset：
   * RST=1 -> 200ms -> 0 -> 2ms -> 1 -> 200ms
   */
  stm32_gpiowrite(priv->gpio_rst, true);
  up_mdelay(200);

  stm32_gpiowrite(priv->gpio_rst, false);
  up_mdelay(2);

  stm32_gpiowrite(priv->gpio_rst, true);
  up_mdelay(200);
}

/****************************************************************************
 * RAM 窗口与指针
 ****************************************************************************/
static void epd_set_window(FAR struct ws29_dev_s *priv,
                           uint16_t xs, uint16_t xe,
                           uint16_t ys, uint16_t ye)
{
  epd_cmd(priv, CMD_SET_RAM_X_ADDRESS);
  uint8_t xdat[2] =
    {
      (uint8_t)((xs >> 3) & 0xFF),
      (uint8_t)((xe >> 3) & 0xFF)
    };
  epd_data(priv, xdat, 2);

  epd_cmd(priv, CMD_SET_RAM_Y_ADDRESS);
  uint8_t ydat[4] =
    {
      (uint8_t)(ys & 0xFF),
      (uint8_t)((ys >> 8) & 0xFF),
      (uint8_t)(ye & 0xFF),
      (uint8_t)((ye >> 8) & 0xFF)
    };
  epd_data(priv, ydat, 4);
}

static void epd_set_cursor(FAR struct ws29_dev_s *priv,
                           uint16_t x, uint16_t y)
{
  epd_cmd(priv, CMD_SET_RAM_X_COUNTER);
  uint8_t xbyte = (uint8_t)((x >> 3) & 0xFF);
  epd_data(priv, &xbyte, 1);

  epd_cmd(priv, CMD_SET_RAM_Y_COUNTER);
  uint8_t ydat[2] =
    {
      (uint8_t)(y & 0xFF),
      (uint8_t)((y >> 8) & 0xFF)
    };
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
      syslog(LOG_ERR, "epaper: full_write len too small: %zu < %zu\n",
             (size_t)len, priv->fbsize);
      return -EINVAL;
    }

  syslog(LOG_DEBUG, "epaper: full_write fbsize=%zu\n", priv->fbsize);

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
  syslog(LOG_INFO, "epaper: clear (white fill)\n");

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
  /* 使用缓存的 update_ctrl_1 / update_ctrl_2
   * 在 epd_init_hw() 里我们把它们设置为 0x00 / 0xF7（强全刷）
   * 若用户通过 IOCTL 修改，则使用用户设置值。
   */

  syslog(LOG_INFO, "epaper: refresh (Display Update, u1=0x%02x,u2=0x%02x)\n",
         priv->update_ctrl_1, priv->update_ctrl_2);

  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_1);
  epd_data(priv, &priv->update_ctrl_1, 1);

  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_2);
  epd_data(priv, &priv->update_ctrl_2, 1);

  epd_cmd(priv, CMD_MASTER_ACTIVATION);
  epd_cmd(priv, CMD_NOP); /* 0xFF, 终止帧读写 */

  return epd_wait_busy(priv, 15000);
}

static int epd_sleep(FAR struct ws29_dev_s *priv)
{
  syslog(LOG_INFO, "epaper: enter deep sleep\n");

  epd_cmd(priv, CMD_DEEP_SLEEP_MODE);
  uint8_t v = 0x01; /* 进入睡眠 */
  epd_data(priv, &v, 1);

  priv->powered = false;
  return OK;
}

/****************************************************************************
 * 局部写入 (BLIT)
 ****************************************************************************/
static void pack_row(uint8_t *dst, uint16_t dst_bytes,
                     const uint8_t *src, uint16_t w_bits,
                     uint8_t shift_left)
{
  memset(dst, 0xFF, dst_bytes); /* 默认全白 */

  for (uint16_t sb = 0; sb < w_bits; sb++)
    {
      uint16_t sbyte = sb / 8u;
      uint8_t  smask = (uint8_t)(0x80u >> (sb & 7u));
      bool black = (src[sbyte] & smask) == 0; /* 0=黑 */

      uint16_t dbit = sb + shift_left;
      uint16_t dbyte = dbit / 8u;
      uint8_t  dmask = (uint8_t)(0x80u >> (dbit & 7u));

      if (dbyte >= dst_bytes)
        {
          break;
        }

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
  if (!bits || w == 0 || h == 0)
    {
      syslog(LOG_ERR, "epaper: blit invalid args (bits=%p, w=%u, h=%u)\n",
             bits, w, h);
      return -EINVAL;
    }

  if (x >= priv->width || y >= priv->height)
    {
      syslog(LOG_ERR, "epaper: blit out of range (x=%u,y=%u)\n", x, y);
      return -EINVAL;
    }

  if ((uint32_t)x + w > priv->width || (uint32_t)y + h > priv->height)
    {
      syslog(LOG_ERR, "epaper: blit region overflow (x=%u,y=%u,w=%u,h=%u)\n",
             x, y, w, h);
      return -EINVAL;
    }

  uint16_t x_aligned = (uint16_t)(x & ~7u);
  uint8_t  shift     = (uint8_t)(x - x_aligned);
  uint16_t dst_bytes = (uint16_t)((shift + w + 7u) / 8u);
  uint16_t src_bpr   = (uint16_t)((w + 7u) / 8u);

  syslog(LOG_DEBUG,
         "epaper: blit x=%u,y=%u,w=%u,h=%u,x_align=%u,shift=%u,dst_bytes=%u,src_bpr=%u\n",
         x, y, w, h, x_aligned, shift, dst_bytes, src_bpr);

  /* 设置窗口覆盖该区域 (按字节对齐扩张) */
  epd_set_window(priv,
                 x_aligned,
                 (uint16_t)(x_aligned + dst_bytes * 8u - 1u),
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
      rowbuf = (uint8_t *)kmm_malloc(dst_bytes);
      if (!rowbuf)
        {
          syslog(LOG_ERR, "epaper: blit kmm_malloc(%u) failed\n",
                 dst_bytes);
          return -ENOMEM;
        }
      dyn = true;
    }

  for (uint16_t row = 0; row < h; row++)
    {
      const uint8_t *src_row = bits + row * src_bpr;
      pack_row(rowbuf, dst_bytes, src_row, w, shift);

      epd_set_cursor(priv, x_aligned, (uint16_t)(y + row));
      epd_cmd(priv, CMD_WRITE_RAM);
      epd_data(priv, rowbuf, dst_bytes);
    }

  if (dyn)
    {
      kmm_free(rowbuf);
    }

  return OK;
}

/****************************************************************************
 * 初始化序列
 ****************************************************************************/
static int epd_init_hw(FAR struct ws29_dev_s *priv)
{
  syslog(LOG_INFO, "epaper: init hw begin\n");

  epd_reset(priv);

  /* Software reset (0x12) */
  epd_cmd(priv, CMD_SWRESET);
  if (epd_wait_busy(priv, 5000) < 0)
    {
      syslog(LOG_ERR, "epaper: SWRESET wait busy timeout\n");
      return -ETIMEDOUT;
    }

  /* DRIVER_OUTPUT_CONTROL (0x01) */
  epd_cmd(priv, CMD_DRIVER_OUTPUT_CONTROL);
  {
    uint8_t doc[3] =
      {
        (uint8_t)((EPD_HEIGHT - 1) & 0xFF),
        (uint8_t)(((EPD_HEIGHT - 1) >> 8) & 0xFF),
        0x00
      };
    epd_data(priv, doc, 3);
  }

  /* BOOSTER_SOFT_START_CONTROL (0x0C) */
  epd_cmd(priv, CMD_BOOSTER_SOFT_START);
  {
    uint8_t bss[3] = { 0xD7, 0xD6, 0x9D };
    epd_data(priv, bss, 3);
  }

  /* WRITE_VCOM_REGISTER (0x2C) 官方示例: 0xA8 */
  priv->vcom = 0xA8;
  epd_cmd(priv, CMD_WRITE_VCOM_REGISTER);
  epd_data(priv, &priv->vcom, 1);

  /* SET_DUMMY_LINE_PERIOD (0x3A) */
  priv->dummy_line = 0x1A;
  epd_cmd(priv, CMD_SET_DUMMY_LINE_PERIOD);
  epd_data(priv, &priv->dummy_line, 1);

  /* SET_GATE_TIME / Gate line width (0x3B) */
  priv->gate_line_w = 0x08;
  epd_cmd(priv, CMD_SET_GATE_LINE_WIDTH);
  epd_data(priv, &priv->gate_line_w, 1);

  /* DATA_ENTRY_MODE_SETTING (0x11) */
  priv->data_entry = 0x03;
  epd_cmd(priv, CMD_DATA_ENTRY_MODE_SETTING);
  epd_data(priv, &priv->data_entry, 1);

  /* BORDER_WAVEFORM_CONTROL (0x3C) 官方示例: 0x03 */
  priv->border_ctrl = 0x03;
  epd_cmd(priv, CMD_BORDER_WAVEFORM_CONTROL);
  epd_data(priv, &priv->border_ctrl, 1);

  /* LUT (0x32) 写入 30 字节: 使用官方 full update LUT */
  memcpy(priv->lut, g_ws29_lut_full, 30);
  epd_cmd(priv, CMD_WRITE_LUT_REGISTER);
  epd_data(priv, priv->lut, 30);

  /* 设置 Display Update Control 1/2:
   * 使用 0x00 / 0xF7（更强的全刷参数）作为缺省
   */
  priv->update_ctrl_1 = 0x00;
  priv->update_ctrl_2 = 0xc4;

  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_1);
  epd_data(priv, &priv->update_ctrl_1, 1);

  epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_2);
  epd_data(priv, &priv->update_ctrl_2, 1);

  /* 设置整屏窗口 */
  epd_set_window(priv, 0, priv->width - 1, 0, priv->height - 1);
  epd_set_cursor(priv, 0, 0);

  /* 初始化阶段执行多次清屏 + 刷新，尽量清掉上电残影 */
  for (int i = 0; i < 2; i++)   /* 不够可以改成 3 试试 */
    {
      syslog(LOG_INFO, "epaper: initial clear+refresh pass %d\n", i);

      int ret = epd_clear(priv);
      if (ret < 0)
        {
          syslog(LOG_ERR, "epaper: initial clear failed: %d\n", ret);
          return ret;
        }

      ret = epd_refresh(priv);
      if (ret < 0)
        {
          syslog(LOG_ERR, "epaper: initial refresh failed: %d\n", ret);
          return ret;
        }
    }

  priv->powered = true;

  syslog(LOG_INFO, "epaper: init hw done\n");
  return OK;
}

/****************************************************************************
 * File ops
 ****************************************************************************/
static int ws29_open(FAR struct file *filep)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;

  syslog(LOG_DEBUG, "epaper: open\n");

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

  syslog(LOG_DEBUG, "epaper: close\n");

  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }

  sem_post(&priv->exclsem);
  return OK;
}

static ssize_t ws29_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  (void)filep;
  (void)buffer;
  (void)buflen;

  syslog(LOG_DEBUG, "epaper: read not supported\n");
  return 0;
}

static ssize_t ws29_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;

  syslog(LOG_INFO, "epaper: write len=%zu, fbsize=%zu\n",
         (size_t)buflen, priv->fbsize);

  if (!buffer || buflen < priv->fbsize)
    {
      syslog(LOG_ERR, "epaper: write invalid buffer/len\n");
      return -EINVAL;
    }

  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }

  int ret = epd_full_write(priv, (FAR const uint8_t *)buffer, buflen);
  if (ret == OK)
    {
      ret = epd_refresh(priv);
    }

  sem_post(&priv->exclsem);

  if (ret == OK)
    {
      return (ssize_t)priv->fbsize;
    }

  syslog(LOG_ERR, "epaper: write failed: %d\n", ret);
  return (ssize_t)ret;
}

static int ws29_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct ws29_dev_s *priv = filep->f_inode->i_private;
  int ret = OK;

  syslog(LOG_DEBUG, "epaper: ioctl cmd=0x%x arg=0x%lx\n", cmd, arg);

  if (sem_wait_unintr(&priv->exclsem) < 0)
    {
      return -errno;
    }

  switch (cmd)
    {
      case EPAPER_IOC_GETINFO:
        {
          FAR struct epaper_info *info =
            (FAR struct epaper_info *)(uintptr_t)arg;

          if (!info)
            {
              ret = -EINVAL;
              break;
            }

          info->width  = priv->width;
          info->height = priv->height;
          info->bpp    = EPD_BPP;

          syslog(LOG_DEBUG,
                 "epaper: GETINFO w=%u,h=%u,bpp=%u\n",
                 info->width, info->height, info->bpp);
        }
        break;

      case EPAPER_IOC_CLEAR:
        syslog(LOG_INFO, "epaper: IOCTL clear\n");
        ret = epd_clear(priv);
        if (ret == OK)
          {
            ret = epd_refresh(priv);
          }
        break;

      case EPAPER_IOC_REFRESH:
        syslog(LOG_INFO, "epaper: IOCTL refresh\n");
        ret = epd_refresh(priv);
        break;

      case EPAPER_IOC_SLEEP:
        syslog(LOG_INFO, "epaper: IOCTL sleep\n");
        ret = epd_sleep(priv);
        break;

      case EPAPER_IOC_BLIT:
        {
          FAR struct epaper_blit *bl =
            (FAR struct epaper_blit *)(uintptr_t)arg;

          if (!bl || !bl->addr)
            {
              syslog(LOG_ERR, "epaper: IOCTL BLIT invalid arg\n");
              ret = -EINVAL;
              break;
            }

          syslog(LOG_INFO,
                 "epaper: IOCTL blit x=%u,y=%u,w=%u,h=%u\n",
                 bl->x, bl->y, bl->w, bl->h);

          ret = epd_blit(priv, bl->x, bl->y, bl->w, bl->h,
                         (FAR const uint8_t *)bl->addr);
        }
        break;

      case EPAPER_IOC_SETLUT:
        {
          FAR const uint8_t *lut =
            (FAR const uint8_t *)(uintptr_t)arg;

          if (!lut)
            {
              syslog(LOG_ERR, "epaper: IOCTL SETLUT lut=NULL\n");
              ret = -EINVAL;
              break;
            }
          memcpy(priv->lut, lut, 30);

          syslog(LOG_INFO, "epaper: IOCTL set LUT\n");

          epd_cmd(priv, CMD_WRITE_LUT_REGISTER);
          epd_data(priv, priv->lut, 30);
        }
        break;

      case EPAPER_IOC_SETBORDER:
        {
          uint8_t v = (uint8_t)arg;
          priv->border_ctrl = v;

          syslog(LOG_INFO, "epaper: IOCTL set border=0x%02x\n", v);

          epd_cmd(priv, CMD_BORDER_WAVEFORM_CONTROL);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETUPDATE1:
        {
          uint8_t v = (uint8_t)arg;
          priv->update_ctrl_1 = v;

          syslog(LOG_INFO, "epaper: IOCTL set update1=0x%02x\n", v);

          epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_1);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETUPDATE2:
        {
          uint8_t v = (uint8_t)arg;
          priv->update_ctrl_2 = v;

          syslog(LOG_INFO, "epaper: IOCTL set update2=0x%02x\n", v);

          epd_cmd(priv, CMD_DISPLAY_UPDATE_CONTROL_2);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETVCOM:
        {
          uint8_t v = (uint8_t)arg;
          priv->vcom = v;

          syslog(LOG_INFO, "epaper: IOCTL set VCOM=0x%02x\n", v);

          epd_cmd(priv, CMD_WRITE_VCOM_REGISTER);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETDUMMY:
        {
          uint8_t v = (uint8_t)arg;
          priv->dummy_line = v;

          syslog(LOG_INFO, "epaper: IOCTL set dummy=0x%02x\n", v);

          epd_cmd(priv, CMD_SET_DUMMY_LINE_PERIOD);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETGATEWIDTH:
        {
          uint8_t v = (uint8_t)arg;
          priv->gate_line_w = v;

          syslog(LOG_INFO, "epaper: IOCTL set gate width=0x%02x\n", v);

          epd_cmd(priv, CMD_SET_GATE_LINE_WIDTH);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_SETDATAENTRY:
        {
          uint8_t v = (uint8_t)arg;
          priv->data_entry = v;

          syslog(LOG_INFO, "epaper: IOCTL set data_entry=0x%02x\n", v);

          epd_cmd(priv, CMD_DATA_ENTRY_MODE_SETTING);
          epd_data(priv, &v, 1);
        }
        break;

      case EPAPER_IOC_TEMPWRITE:
        {
          FAR const uint8_t *tp =
            (FAR const uint8_t *)(uintptr_t)arg;

          if (!tp)
            {
            syslog(LOG_ERR, "epaper: IOCTL TEMPWRITE tp=NULL\n");
              ret = -EINVAL;
              break;
            }

          syslog(LOG_INFO,
                 "epaper: IOCTL TEMPWRITE %02x %02x\n",
                 tp[0], tp[1]);

          epd_cmd(priv, CMD_TEMP_SENSOR_CONTROL);
          epd_data(priv, tp, 2);
        }
        break;

      default:
        syslog(LOG_ERR, "epaper: IOCTL unknown cmd=0x%x\n", cmd);
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
      syslog(LOG_ERR, "epaper: initialize with NULL spi\n");
      return -ENODEV;
    }

  priv = (FAR struct ws29_dev_s *)kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      syslog(LOG_ERR, "epaper: kmm_zalloc ws29_dev_s failed\n");
      return -ENOMEM;
    }

  priv->spi       = spi;
  priv->gpio_busy = gpio_busy;
  priv->gpio_rst  = gpio_rst;
  priv->width     = EPD_WIDTH;
  priv->height    = EPD_HEIGHT;
  priv->fbsize    = EPD_FB_SIZE;
  priv->powered   = false;

  priv->fb = NULL;

  sem_init(&priv->exclsem, 0, 1);

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, 4000000);

  syslog(LOG_INFO,
         "epaper: init SPI done, busy=0x%08x, rst=0x%08x\n",
         gpio_busy, gpio_rst);

  ret = epd_init_hw(priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "epaper: init hw failed: %d\n", ret);
      sem_destroy(&priv->exclsem);
      if (priv->fb)
        {
          kmm_free(priv->fb);
        }
      kmm_free(priv);
      return ret;
    }

  ret = register_driver("/dev/epaper", &g_epaper_fops, 0644, priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "epaper: register_driver failed: %d\n", ret);
      sem_destroy(&priv->exclsem);
      if (priv->fb)
        {
          kmm_free(priv->fb);
        }
      kmm_free(priv);
      return ret;
    }

  syslog(LOG_INFO,
         "epaper: /dev/epaper registered (%ux%u, fbsize=%u)\n",
         priv->width, priv->height, (unsigned)priv->fbsize);

  return OK;
}
