/****************************************************************************
 * Waveshare 2.9" ePaper 板级初始化 (使用 SPI CMD/DATA 回调)
 ****************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"

#include <nuttx/epaper/waveshare_2_9.h>

struct epd_board_ctx_s
{
  FAR struct spi_dev_s *spi;
  bool                  initialized;
  int                   spino;
};

static struct epd_board_ctx_s g_epd_ctx = {0};

#ifndef EPD_SPI_FREQUENCY
#  define EPD_SPI_FREQUENCY 4000000
#endif
#ifndef EPD_SPI_BITS
#  define EPD_SPI_BITS 8
#endif
#ifndef EPD_SPI_MODE
#  define EPD_SPI_MODE SPIDEV_MODE0
#endif

int stm32_epaper_init(int spi_port)
{
  int ret;

  if (g_epd_ctx.initialized)
    {
      syslog(LOG_INFO, "ePaper already initialized (SPI%d)\n", g_epd_ctx.spino);
      return OK;
    }

  g_epd_ctx.spino = spi_port;

  /* BUSY / RST GPIO */
  stm32_configgpio(GPIO_EPD_BUSY);
  stm32_configgpio(GPIO_EPD_RST);
  /* DC / CS 已在 stm32_spidev_initialize() 中配置 */

  FAR struct spi_dev_s *spi = stm32_spibus_initialize(spi_port);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: stm32_spibus_initialize(%d) failed\n", spi_port);
      return -ENODEV;
    }
  g_epd_ctx.spi = spi;

  /* 这些宏是语句宏，不要加 (void) 强制转换 */
  SPI_SETMODE(spi, EPD_SPI_MODE);
  SPI_SETBITS(spi, EPD_SPI_BITS);
  SPI_SETFREQUENCY(spi, EPD_SPI_FREQUENCY);

  /* 使用 SPI CMDDATA，不再传 DC 引脚 */
  ret = ws_epaper_initialize(spi, GPIO_EPD_BUSY, GPIO_EPD_RST);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ws_epaper_initialize failed: %d\n", ret);
      return ret;
    }

  g_epd_ctx.initialized = true;

  syslog(LOG_INFO, "Waveshare 2.9\" ePaper init OK on SPI%d\n", spi_port);
  return OK;
}

int stm32_epaper_deinit(void)
{
  if (!g_epd_ctx.initialized)
    {
      return -EALREADY;
    }

  g_epd_ctx.initialized = false;
  g_epd_ctx.spi         = NULL;
  g_epd_ctx.spino       = -1;

  syslog(LOG_INFO, "ePaper deinitialized\n");
  return OK;
}
