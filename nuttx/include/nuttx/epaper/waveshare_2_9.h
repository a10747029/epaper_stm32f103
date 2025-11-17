/****************************************************************************
 * Public API for Waveshare 2.9" ePaper
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_EPAPER_WAVESHARE_2_9_H
#define __INCLUDE_NUTTX_EPAPER_WAVESHARE_2_9_H

#include <nuttx/spi/spi.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化接口：使用 SPI CMDDATA，不再需要 DC 参数 */
int ws_epaper_initialize(FAR struct spi_dev_s *spi,
                         uint32_t gpio_busy,
                         uint32_t gpio_rst);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_EPAPER_WAVESHARE_2_9_H */
