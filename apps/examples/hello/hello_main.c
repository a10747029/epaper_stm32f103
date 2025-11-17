/****************************************************************************
 * apps/examples/hello/hello_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
/****************************************************************************
 * apps/examples/epaper_hello/epaper_hello_main.c
 *
 * Demo app for Waveshare 2.9" ePaper:
 * - Open /dev/epaper
 * - Get resolution by ioctl (fallback to 128x296)
 * - Clear screen
 * - Render 24x24 seven-segment digits "12345" (1bpp)
 * - BLIT each glyph to panel via ioctl
 * - Refresh panel
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

/* Keep in sync with your driver (drivers/epaper/waveshare_2_9.c)
 * Existing ioctls there: CLEAR=0xE3, REFRESH=0xE1, SLEEP=0xE2 (as examples).
 * Here we define two new ioctls that the app expects; please add them in driver later.
 */
#ifndef EPAPER_IOC_BASE
#  define EPAPER_IOC_BASE        0xE0
#endif
#ifndef EPAPER_IOC_REFRESH
#  define EPAPER_IOC_REFRESH     (EPAPER_IOC_BASE + 1)
#endif
#ifndef EPAPER_IOC_SLEEP
#  define EPAPER_IOC_SLEEP       (EPAPER_IOC_BASE + 2)
#endif
#ifndef EPAPER_IOC_CLEAR
#  define EPAPER_IOC_CLEAR       (EPAPER_IOC_BASE + 3)
#endif

/* New ioctls for this app */
#ifndef EPAPER_IOC_GETINFO
#  define EPAPER_IOC_GETINFO     (EPAPER_IOC_BASE + 10)
#endif
#ifndef EPAPER_IOC_BLIT
#  define EPAPER_IOC_BLIT        (EPAPER_IOC_BASE + 11)
#endif

/* Structures for GETINFO/BLIT */
struct epaper_info
{
  uint16_t width;   /* pixels */
  uint16_t height;  /* pixels */
  uint8_t  bpp;     /* bits per pixel (expect 1) */
  uint8_t  reserved;
};

struct epaper_blit
{
  uint16_t x;       /* dest x (pixels) */
  uint16_t y;       /* dest y (pixels) */
  uint16_t w;       /* width  (pixels) */
  uint16_t h;       /* height (pixels) */
  FAR const void *addr; /* pointer to packed 1bpp bitmap (MSB first per byte) */
};

/* Glyph size: 24x24 => 72 bytes */
#define GLYPH_W     24
#define GLYPH_H     24
#define GLYPH_BPR   (GLYPH_W/8)   /* bytes per row = 3 */
#define GLYPH_SIZE  (GLYPH_H*GLYPH_BPR)

/* Pack bit helpers (1=white, 0=black typical for EPD RAM) */
static inline void glyph_fill_white(uint8_t *buf)
{
  memset(buf, 0xFF, GLYPH_SIZE);
}

static inline void glyph_set_pixel(uint8_t *buf, int x, int y, bool black)
{
  if (x < 0 || x >= GLYPH_W || y < 0 || y >= GLYPH_H)
    {
      return;
    }
  int idx = y * GLYPH_BPR + (x >> 3);
  uint8_t mask = (uint8_t)(0x80u >> (x & 7));

  if (black)
    {
      /* 0 = black on typical ePaper RAM */
      buf[idx] &= (uint8_t)(~mask);
    }
  else
    {
      buf[idx] |= mask;
    }
}

/* Draw thick horizontal/vertical lines for seven-segment style */
static void draw_hline(uint8_t *buf, int x0, int x1, int y, int thick)
{
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  for (int t = 0; t < thick; t++)
    {
      int yy = y + t;
      if (yy < 0 || yy >= GLYPH_H) continue;
      for (int x = x0; x <= x1; x++)
        {
          glyph_set_pixel(buf, x, yy, true);
        }
    }
}

static void draw_vline(uint8_t *buf, int x, int y0, int y1, int thick)
{
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
  for (int t = 0; t < thick; t++)
    {
      int xx = x + t;
      if (xx < 0 || xx >= GLYPH_W) continue;
      for (int y = y0; y <= y1; y++)
        {
          glyph_set_pixel(buf, xx, y, true);
        }
    }
}

/* Seven-segment layout (within 24x24; margins for readability)
   Segments: a(top), b(top-right), c(bottom-right), d(bottom),
             e(bottom-left), f(top-left), g(middle)
*/
struct segments
{
  bool a, b, c, d, e, f, g;
};

static struct segments digit_to_segments(int d)
{
  /* standard seven-seg */
  switch (d)
    {
      case 0: return (struct segments){true,  true,  true,  true,  true,  true,  false};
      case 1: return (struct segments){false, true,  true,  false, false, false, false};
      case 2: return (struct segments){true,  true,  false, true,  true,  false, true };
      case 3: return (struct segments){true,  true,  true,  true,  false, false, true };
      case 4: return (struct segments){false, true,  true,  false, false, true,  true };
      case 5: return (struct segments){true,  false, true,  true,  false, true,  true };
      case 6: return (struct segments){true,  false, true,  true,  true,  true,  true };
      case 7: return (struct segments){true,  true,  true,  false, false, false, false};
      case 8: return (struct segments){true,  true,  true,  true,  true,  true,  true };
      case 9: return (struct segments){true,  true,  true,  true,  false, true,  true };
      default:return (struct segments){false, false, false, false, false, false, false};
    }
}

static void render_digit_24x24(int d, uint8_t *out /*72B*/)
{
  /* Geometry parameters */
  const int thick = 3;      /* segment thickness */
  const int xm = 2;         /* left margin  */
  const int ym = 2;         /* top margin   */
  const int xR = GLYPH_W - 3;  /* right inner ref (~21) */
  const int yB = GLYPH_H - 3;  /* bottom inner ref (~21) */
  const int yM = GLYPH_H/2 - 1;/* middle band start (~11) */

  glyph_fill_white(out);

  struct segments s = digit_to_segments(d);

  if (s.a) draw_hline(out, xm, xR, ym, thick);            /* top */
  if (s.g) draw_hline(out, xm, xR, yM, thick);            /* middle */
  if (s.d) draw_hline(out, xm, xR, yB, thick);            /* bottom */

  if (s.f) draw_vline(out, xm, ym, yM-1, thick);          /* top-left */
  if (s.b) draw_vline(out, xR, ym, yM-1, thick);          /* top-right */
  if (s.e) draw_vline(out, xm, yM+1, yB, thick);          /* bottom-left */
  if (s.c) draw_vline(out, xR, yM+1, yB, thick);          /* bottom-right */
}

int main(int argc, FAR char *argv[])
{
  const char *dev = "/dev/epaper";
  int fd = open(dev, O_RDWR);
  if (fd < 0)
    {
      printf("open %s failed: %d\n", dev, errno);
      return 1;
    }

  /* 1) Get display info */
  struct epaper_info info;
  memset(&info, 0, sizeof(info));
  if (ioctl(fd, EPAPER_IOC_GETINFO, (unsigned long)(uintptr_t)&info) < 0)
    {
      /* Fallback if driver not yet implements GETINFO */
      info.width  = 128;
      info.height = 296;
      info.bpp    = 1;
      printf("GETINFO not supported, fallback to %ux%u\n",
             info.width, info.height);
    }
  else
    {
      printf("Panel: %ux%u, %u bpp\n", info.width, info.height, info.bpp);
    }

  /* 2) Clear screen */
  while(ioctl(fd, EPAPER_IOC_CLEAR, 0) < 0)
  {
	  printf("CLEAR ioctl failed: %d (continuing)\n", errno);
	  sleep(1);
  }
  if (ioctl(fd, EPAPER_IOC_CLEAR, 0) < 0)
    {
      printf("CLEAR ioctl failed: %d (continuing)\n", errno);
    }

  /* 3) Prepare and draw "12345" */
  const char *text = "12345";
  const int   n    = 5;

  /* Compute start position: horizontally centered for 5 glyphs (each 24 wide) */
  int totalw = n * GLYPH_W;
  int startx = (int)info.width > totalw ? ((int)info.width - totalw)/2 : 0;
  /* Vertically center */
  int starty = (int)info.height > GLYPH_H ? ((int)info.height - GLYPH_H)/2 : 0;

  for (int i = 0; i < n; i++)
    {
      char ch = text[i];
      if (ch < '0' || ch > '9')
        {
          continue;
        }
      int digit = ch - '0';

      uint8_t glyph[GLYPH_SIZE];
      render_digit_24x24(digit, glyph);

      struct epaper_blit blit;
      blit.x    = (uint16_t)(startx + i * GLYPH_W);
      blit.y    = (uint16_t)starty;
      blit.w    = GLYPH_W;
      blit.h    = GLYPH_H;
      blit.addr = (FAR const void *)glyph;

      if (ioctl(fd, EPAPER_IOC_BLIT, (unsigned long)(uintptr_t)&blit) < 0)
        {
          printf("BLIT ioctl failed at i=%d: %d\n", i, errno);
          /* 如果驱动暂未支持 BLIT，可在这里退化为 write 全屏或略过 */
        }
    }

  /* 4) Refresh */
  if (ioctl(fd, EPAPER_IOC_REFRESH, 0) < 0)
    {
      printf("REFRESH ioctl failed: %d\n", errno);
    }

  close(fd);
  return 0;
}
