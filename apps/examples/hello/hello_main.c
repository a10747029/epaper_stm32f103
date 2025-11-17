/****************************************************************************
 * apps/examples/hello/hello_main.c
 *
 * Waveshare 2.9" ePaper demo:
 * - Open /dev/epaper
 * - Get resolution by ioctl (expect 128x296)
 * - Clear screen
 * - Auto-select font size (SxS, S is multiple of 12) based on char count
 * - Vertically render hex-like digits "0-9,A-F" (extendable to A-Z)
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

/* IOCTLs, keep in sync with waveshare_2_9.c */
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
#ifndef EPAPER_IOC_GETINFO
#  define EPAPER_IOC_GETINFO     (EPAPER_IOC_BASE + 10)
#endif
#ifndef EPAPER_IOC_BLIT
#  define EPAPER_IOC_BLIT        (EPAPER_IOC_BASE + 11)
#endif

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
  FAR const void *addr;
};

/* 我们将 glyph 尺寸参数化为 SxS，S 为 12 的倍数。最大支持 128。 */
#define GLYPH_MAX  128
#define GLYPH_MAX_BPR   (GLYPH_MAX/8)       /* bytes per row (for S=128) */
#define GLYPH_MAX_SIZE  (GLYPH_MAX*GLYPH_MAX_BPR)

/* 1=white, 0=black */
static inline void glyph_fill_white(uint8_t *buf, int S)
{
  int bpr  = S / 8;
  int size = S * bpr;
  memset(buf, 0xFF, size);
}

static inline void glyph_set_pixel(uint8_t *buf, int S, int x, int y, bool black)
{
  if (x < 0 || x >= S || y < 0 || y >= S)
    {
      return;
    }

  int bpr = S / 8;
  int idx = y * bpr + (x >> 3);
  uint8_t mask = (uint8_t)(0x80u >> (x & 7));

  if (black)
    {
      buf[idx] &= (uint8_t)(~mask);
    }
  else
    {
      buf[idx] |= mask;
    }
}

static void draw_hline(uint8_t *buf, int S, int x0, int x1, int y, int thick)
{
  if (x0 > x1)
    {
      int t = x0; x0 = x1; x1 = t;
    }
  for (int t = 0; t < thick; t++)
    {
      int yy = y + t;
      if (yy < 0 || yy >= S) continue;
      for (int x = x0; x <= x1; x++)
        {
          glyph_set_pixel(buf, S, x, yy, true);
        }
    }
}

static void draw_vline(uint8_t *buf, int S, int x, int y0, int y1, int thick)
{
  if (y0 > y1)
    {
      int t = y0; y0 = y1; y1 = t;
    }
  for (int t = 0; t < thick; t++)
    {
      int xx = x + t;
      if (xx < 0 || xx >= S) continue;
      for (int y = y0; y <= y1; y++)
        {
          glyph_set_pixel(buf, S, xx, y, true);
        }
    }
}

/* 简化版“多段段码管”（主要是加强 7 段，对 0-9,A-F 足够清晰） */
struct segx
{
  bool a;   /* top */
  bool b;   /* upper-right */
  bool c;   /* lower-right */
  bool d;   /* bottom */
  bool e;   /* lower-left */
  bool f;   /* upper-left */
  bool g1;  /* upper-middle */
  bool g2;  /* lower-middle */
  /* 预留 h,i,j,k,l,m 做以后 14 段扩展 */
};

static struct segx char_to_segments(char ch)
{
  switch (ch)
    {
      case '0': return (struct segx){true,  true,  true,  true,  true,  true,  false, false};
      case '1': return (struct segx){false, true,  true,  false, false, false, false, false};
      case '2': return (struct segx){true,  true,  false, true,  true,  false, true,  false};
      case '3': return (struct segx){true,  true,  true,  true,  false, false, true,  false};
      case '4': return (struct segx){false, true,  true,  false, false, true,  true,  false};
      case '5': return (struct segx){true,  false, true,  true,  false, true,  true,  false};
      case '6': return (struct segx){true,  false, true,  true,  true,  true,  true,  false};
      case '7': return (struct segx){true,  true,  true,  false, false, false, false, false};
      case '8': return (struct segx){true,  true,  true,  true,  true,  true,  true,  false};
      case '9': return (struct segx){true,  true,  true,  true,  false, true,  true,  false};

      /* 16 进制字母示例：A-F，你可以按这个模式继续扩到 A-Z */
      case 'A': return (struct segx){true,  true,  true,  false, true,  true,  true,  false};
      case 'B': /* 近似显示为 8 左下略弱 */
               return (struct segx){false, true,  true,  true,  true,  true,  true,  false};
      case 'C': return (struct segx){true,  false, false, true,  true,  true,  false, false};
      case 'D': return (struct segx){false, true,  true,  true,  true,  false, true,  false};
      case 'E': return (struct segx){true,  false, false, true,  true,  true,  true,  false};
      case 'F': return (struct segx){true,  false, false, false, true,  true,  true,  false};

      default:  return (struct segx){false,false,false,false,false,false,false,false};
    }
}

/* 根据 S 绘制一个 segx 字符 */
static void render_segx_char(struct segx s, int S, uint8_t *buf)
{
  glyph_fill_white(buf, S);

  int thick = S / 12;                 /* 线宽 */
  if (thick < 2) thick = 2;

  int xm = S / 6;                     /* 左右边距 */
  int xR = S - 1 - xm;

  int yt = S / 8;                     /* top */
  int yb = S - 1 - yt;                /* bottom */
  int yM = S / 2 - thick;            /* middle band */

  /* a,d,g1,g2 */
  if (s.a)  draw_hline(buf, S, xm, xR, yt, thick);
  if (s.d)  draw_hline(buf, S, xm, xR, yb, thick);
  if (s.g1) draw_hline(buf, S, xm, xR, yM, thick);
  if (s.g2) draw_hline(buf, S, xm, xR, yM + thick + 1, thick);

  /* f,b,e,c */
  int upper_end = yM - 1;
  int lower_start = yM + thick + 1;

  if (s.f) draw_vline(buf, S, xm, yt, upper_end, thick);
  if (s.b) draw_vline(buf, S, xR, yt, upper_end, thick);
  if (s.e) draw_vline(buf, S, xm, lower_start, yb, thick);
  if (s.c) draw_vline(buf, S, xR, lower_start, yb, thick);
}

/* 选择字体尺寸 S (12 的倍数)，竖排布局时高度优先 */
static int choose_font_size(int width, int height, int n_chars)
{
  const int step = 12;
  const int gap  = 4;

  int max_h_per = (height - (n_chars - 1) * gap) / n_chars;
  if (max_h_per <= 0) max_h_per = step;

  int limit = max_h_per < width ? max_h_per : width;

  if (limit < step) return step;

  int S = (limit / step) * step;
  if (S > GLYPH_MAX) S = GLYPH_MAX;
  return S;
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

  /* 1) 读取面板信息 */
  struct epaper_info info;
  memset(&info, 0, sizeof(info));
  if (ioctl(fd, EPAPER_IOC_GETINFO, (unsigned long)(uintptr_t)&info) < 0)
    {
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

  /* 2) 清屏 */
  while (ioctl(fd, EPAPER_IOC_CLEAR, 0) < 0)
    {
      printf("CLEAR ioctl failed: %d (retry in 1s)\n", errno);
      sleep(1);
    }

  /* 3) 决定要显示的字符串：
   *    - 若用户通过 NSH 参数传入，则用 argv[1]；
   *    - 否则默认 "12345"。
   *    - 只保留 0-9 和 A-F （你后面可以扩展到 A-Z）。
   */
  char textbuf[32];
  const char *src = (argc > 1 && argv[1] && argv[1][0]) ? argv[1] : "12345";
  int n = 0;
  for (const char *p = src; *p && n < (int)sizeof(textbuf)-1; p++)
    {
      char ch = *p;
      if (ch >= 'a' && ch <= 'f') ch = (char)(ch - 'a' + 'A');
      if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F'))
        {
          textbuf[n++] = ch;
        }
    }
  if (n == 0)
    {
      textbuf[0] = '0';
      n = 1;
    }
  textbuf[n] = '\0';

  /* 4) 根据字符数 & 面板大小选字体尺寸 */
  int S = choose_font_size(info.width, info.height, n);
  printf("Using font size: %d x %d for %d chars\n", S, S, n);

  /* 5) 计算竖排布局的起始坐标 */
  const int gap = 4;

  int used_h = n * S + (n - 1) * gap;
  int starty = (int)info.height > used_h ? ((int)info.height - used_h) / 2 : 0;
  int startx = (int)info.width  > S     ? ((int)info.width  - S)        / 2 : 0;

  /* 6) 为每个字符生成 segx 字形并 BLIT */
  uint8_t glyph_buf[GLYPH_MAX_SIZE]; /* 足够容纳 S<=128 的 glyph */

  for (int i = 0; i < n; i++)
    {
      char ch = textbuf[i];
      struct segx seg = char_to_segments(ch);
      render_segx_char(seg, S, glyph_buf);

      struct epaper_blit blit;
      blit.x    = (uint16_t)startx;
      blit.y    = (uint16_t)(starty + i * (S + gap));
      blit.w    = (uint16_t)S;
      blit.h    = (uint16_t)S;
      blit.addr = (FAR const void *)glyph_buf;

      if (ioctl(fd, EPAPER_IOC_BLIT, (unsigned long)(uintptr_t)&blit) < 0)
        {
          printf("BLIT ioctl failed at i=%d (%c): %d\n", i, ch, errno);
        }
    }

  /* 7) 刷新显示 */
  if (ioctl(fd, EPAPER_IOC_REFRESH, 0) < 0)
    {
      printf("REFRESH ioctl failed: %d\n", errno);
    }

  close(fd);
  return 0;
}
