#ifndef C30D_OLED_H
#define C30D_OLED_H
#include <stdint.h>
void C30D_OledInit(void);
/* One page per text row; clips to 21 columns and replaces unsupported glyphs. */
void C30D_OledLine(unsigned row, const char *text);
void C30D_OledRefresh(void);
/* Owned by monitorTask; debugger may inspect when halted. No panel readback. */
extern uint8_t c30d_oled_framebuffer[8][128];
#endif
