/*******************************************************************************
 * Size: 60 px
 * Bpp: 1
 * Opts: 
 ******************************************************************************/

#define LV_LVGL_H_INCLUDE_SIMPLE

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl.h"
#endif

#ifndef DSEG60
#define DSEG60 1
#endif

#if DSEG60

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+002D "-" */
    0x1f, 0xff, 0xff, 0xf1, 0xff, 0xff, 0xff, 0x9f,
    0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xc7, 0xff,
    0xff, 0xfc, 0x0,

    /* U+002E "." */
    0x38, 0xfb, 0xff, 0xff, 0xef, 0x8e, 0x0,

    /* U+0030 "0" */
    0x7f, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff,
    0xfc, 0xe7, 0xff, 0xff, 0xff, 0xef, 0xcf, 0xff,
    0xff, 0xfe, 0xff, 0x9f, 0xff, 0xff, 0xe7, 0xfe,
    0x0, 0x0, 0x0, 0x7f, 0xf0, 0x0, 0x0, 0x3,
    0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0, 0x0,
    0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0,
    0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff,
    0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0,
    0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0,
    0x0, 0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0,
    0x0, 0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f,
    0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0,
    0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0,
    0x0, 0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc,
    0x0, 0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf,
    0xff, 0x0, 0x0, 0x0, 0x3f, 0xf8, 0x0, 0x0,
    0x0, 0x7f, 0x80, 0x0, 0x0, 0x0, 0xfc, 0x0,
    0x0, 0x0, 0x30, 0xc0, 0x0, 0x0, 0x3, 0xf0,
    0x0, 0x0, 0x0, 0x1f, 0xe0, 0x0, 0x0, 0x1,
    0xff, 0xc0, 0x0, 0x0, 0xf, 0xff, 0x0, 0x0,
    0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0,
    0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff,
    0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0, 0x0,
    0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0, 0x0,
    0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f, 0xf8,
    0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0, 0x1f,
    0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0, 0x0,
    0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0,
    0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff,
    0x0, 0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3,
    0xff, 0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0,
    0x0, 0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0xbf,
    0xff, 0xff, 0x9f, 0xfb, 0xff, 0xff, 0xff, 0x3f,
    0x9f, 0xff, 0xff, 0xfe, 0x7d, 0xff, 0xff, 0xff,
    0xfc, 0xdf, 0xff, 0xff, 0xff, 0xe0,

    /* U+0031 "1" */
    0x8, 0x31, 0xc7, 0x3d, 0xf7, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf3, 0xc3, 0x4, 0xe7, 0xdf,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xcf, 0xc, 0x0,

    /* U+0032 "2" */
    0x7f, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff,
    0xfc, 0xc7, 0xff, 0xff, 0xff, 0xee, 0xf, 0xff,
    0xff, 0xfe, 0xf0, 0x1f, 0xff, 0xff, 0xe7, 0x80,
    0x0, 0x0, 0x0, 0x7c, 0x0, 0x0, 0x0, 0x3,
    0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0, 0x0,
    0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0,
    0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0,
    0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0,
    0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0,
    0x0, 0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0,
    0x0, 0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e,
    0x0, 0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0,
    0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0,
    0x0, 0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0,
    0x0, 0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf,
    0xc0, 0x0, 0x0, 0x0, 0x1e, 0x3, 0xff, 0xff,
    0xfe, 0x30, 0x3f, 0xff, 0xff, 0xf0, 0x1, 0xff,
    0xff, 0xff, 0x0, 0x1f, 0xff, 0xff, 0xf8, 0x18,
    0xff, 0xff, 0xff, 0x80, 0xf0, 0x0, 0x0, 0x0,
    0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0,
    0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0,
    0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3,
    0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0,
    0x0, 0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0, 0x0,
    0x0, 0x0, 0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8,
    0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0, 0x0,
    0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0, 0x0,
    0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc, 0x0,
    0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f,
    0x0, 0x0, 0x0, 0x1, 0xf8, 0x0, 0x0, 0x0,
    0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0, 0x0,
    0x0, 0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f, 0xbf,
    0xff, 0xff, 0x80, 0xfb, 0xff, 0xff, 0xff, 0x7,
    0x9f, 0xff, 0xff, 0xfe, 0x3d, 0xff, 0xff, 0xff,
    0xfc, 0xdf, 0xff, 0xff, 0xff, 0xe0,

    /* U+0033 "3" */
    0xff, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff,
    0xfc, 0xc7, 0xff, 0xff, 0xff, 0xee, 0xf, 0xff,
    0xff, 0xfe, 0xf0, 0x1f, 0xff, 0xff, 0xe7, 0x80,
    0x0, 0x0, 0x0, 0x7c, 0x0, 0x0, 0x0, 0x3,
    0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0, 0x0,
    0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0,
    0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0,
    0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0,
    0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0,
    0x0, 0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0,
    0x0, 0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e,
    0x0, 0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0,
    0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0,
    0x0, 0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0,
    0x0, 0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf,
    0xc0, 0x0, 0x0, 0x0, 0x1e, 0x7, 0xff, 0xff,
    0xfe, 0x30, 0x7f, 0xff, 0xff, 0xf4, 0x3, 0xff,
    0xff, 0xff, 0x38, 0x3f, 0xff, 0xff, 0xfb, 0xe1,
    0xff, 0xff, 0xff, 0xbf, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0, 0x0,
    0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0,
    0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f,
    0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0, 0x0,
    0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0,
    0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0,
    0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3,
    0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0,
    0x0, 0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0, 0x7f,
    0xff, 0xff, 0x9f, 0x7, 0xff, 0xff, 0xff, 0x38,
    0x3f, 0xff, 0xff, 0xfe, 0x43, 0xff, 0xff, 0xff,
    0xfc, 0x1f, 0xff, 0xff, 0xff, 0xe0,

    /* U+0034 "4" */
    0x0, 0x0, 0x0, 0x0, 0x10, 0x0, 0x0, 0x0,
    0x0, 0xe0, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0,
    0x0, 0x0, 0x7f, 0x80, 0x0, 0x0, 0x7, 0xfe,
    0x0, 0x0, 0x0, 0x7f, 0xf0, 0x0, 0x0, 0x3,
    0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0, 0x0,
    0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0,
    0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff,
    0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0,
    0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0,
    0x0, 0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0,
    0x0, 0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f,
    0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0,
    0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0,
    0x0, 0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc,
    0x0, 0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf,
    0xff, 0x0, 0x0, 0x0, 0x1f, 0xfb, 0xff, 0xff,
    0xfe, 0x3f, 0xbf, 0xff, 0xff, 0xf4, 0x39, 0xff,
    0xff, 0xff, 0x38, 0x5f, 0xff, 0xff, 0xfb, 0xe0,
    0xff, 0xff, 0xff, 0xbf, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0, 0x0,
    0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0,
    0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f,
    0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0, 0x0,
    0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0,
    0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0,
    0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3,
    0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0,
    0x0, 0xfc, 0x0, 0x0, 0x0, 0x3, 0xe0, 0x0,
    0x0, 0x0, 0x7, 0x0, 0x0, 0x0, 0x0, 0x8,

    /* U+0035 "5" */
    0x7f, 0xff, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff,
    0xfc, 0x27, 0xff, 0xff, 0xff, 0xe1, 0xcf, 0xff,
    0xff, 0xfe, 0xf, 0x9f, 0xff, 0xff, 0xe0, 0x7e,
    0x0, 0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0,
    0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0,
    0x0, 0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0,
    0x0, 0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf,
    0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0,
    0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0,
    0x0, 0x0, 0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0,
    0x0, 0x0, 0x0, 0x3f, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xfb, 0xff, 0xff,
    0xfe, 0xf, 0xbf, 0xff, 0xff, 0xf4, 0x39, 0xff,
    0xff, 0xff, 0x38, 0x5f, 0xff, 0xff, 0xfb, 0xe0,
    0xff, 0xff, 0xff, 0xbf, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0, 0x0,
    0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0,
    0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f,
    0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0, 0x0,
    0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0,
    0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0,
    0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3,
    0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0,
    0x0, 0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0, 0x3f,
    0xff, 0xff, 0x9f, 0x3, 0xff, 0xff, 0xff, 0x38,
    0x1f, 0xff, 0xff, 0xfe, 0x41, 0xff, 0xff, 0xff,
    0xfc, 0x1f, 0xff, 0xff, 0xff, 0xe0,

    /* U+0036 "6" */
    0x7f, 0xff, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff,
    0xfc, 0x27, 0xff, 0xff, 0xff, 0xe1, 0xcf, 0xff,
    0xff, 0xfe, 0xf, 0x9f, 0xff, 0xff, 0xe0, 0x7e,
    0x0, 0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0,
    0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0,
    0x0, 0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0,
    0x0, 0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf,
    0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0,
    0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0,
    0x0, 0x0, 0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0,
    0x0, 0x0, 0x0, 0x3f, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xfb, 0xff, 0xff,
    0xfe, 0xf, 0xbf, 0xff, 0xff, 0xf4, 0x39, 0xff,
    0xff, 0xff, 0x38, 0x5f, 0xff, 0xff, 0xfb, 0xf8,
    0xff, 0xff, 0xff, 0xbf, 0xf0, 0x0, 0x0, 0x1,
    0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0, 0x0,
    0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0,
    0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff,
    0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0, 0x0,
    0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0, 0x0,
    0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f, 0xf8,
    0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0, 0x1f,
    0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0, 0x0,
    0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0,
    0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff,
    0x0, 0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3,
    0xff, 0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0,
    0x0, 0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0xbf,
    0xff, 0xff, 0x9f, 0xfb, 0xff, 0xff, 0xff, 0x3f,
    0x9f, 0xff, 0xff, 0xfe, 0x7d, 0xff, 0xff, 0xff,
    0xfc, 0xdf, 0xff, 0xff, 0xff, 0xe0,

    /* U+0037 "7" */
    0x7f, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff,
    0xfc, 0xe7, 0xff, 0xff, 0xff, 0xef, 0xcf, 0xff,
    0xff, 0xfe, 0xff, 0x9f, 0xff, 0xff, 0xe7, 0xfe,
    0x0, 0x0, 0x0, 0x7f, 0xf0, 0x0, 0x0, 0x3,
    0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0, 0x0,
    0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0,
    0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff,
    0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0,
    0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0,
    0x0, 0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0,
    0x0, 0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f,
    0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0,
    0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0,
    0x0, 0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc,
    0x0, 0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf,
    0xff, 0x0, 0x0, 0x0, 0x1f, 0xf8, 0x0, 0x0,
    0x0, 0x3f, 0x80, 0x0, 0x0, 0x4, 0x38, 0x0,
    0x0, 0x0, 0x38, 0x40, 0x0, 0x0, 0x3, 0xe0,
    0x0, 0x0, 0x0, 0x1f, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0, 0x0,
    0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0,
    0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f,
    0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0, 0x0,
    0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0,
    0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0,
    0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3,
    0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0,
    0x0, 0xfc, 0x0, 0x0, 0x0, 0x3, 0xe0, 0x0,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x0, 0x0, 0x18,

    /* U+0038 "8" */
    0x7f, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff,
    0xfc, 0xe7, 0xff, 0xff, 0xff, 0xef, 0xcf, 0xff,
    0xff, 0xfe, 0xff, 0x9f, 0xff, 0xff, 0xe7, 0xfe,
    0x0, 0x0, 0x0, 0x7f, 0xf0, 0x0, 0x0, 0x3,
    0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0, 0x0,
    0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0,
    0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff,
    0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0,
    0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0,
    0x0, 0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0,
    0x0, 0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f,
    0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0,
    0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0,
    0x0, 0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc,
    0x0, 0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf,
    0xff, 0x0, 0x0, 0x0, 0x1f, 0xfb, 0xff, 0xff,
    0xfe, 0x3f, 0xbf, 0xff, 0xff, 0xf4, 0x39, 0xff,
    0xff, 0xff, 0x38, 0x5f, 0xff, 0xff, 0xfb, 0xf8,
    0xff, 0xff, 0xff, 0xbf, 0xf0, 0x0, 0x0, 0x1,
    0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0, 0x0,
    0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0,
    0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff,
    0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0, 0x0,
    0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0, 0x0,
    0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f, 0xf8,
    0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0, 0x1f,
    0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0, 0x0,
    0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0,
    0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff,
    0x0, 0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3,
    0xff, 0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0,
    0x0, 0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0xbf,
    0xff, 0xff, 0x9f, 0xfb, 0xff, 0xff, 0xff, 0x3f,
    0x9f, 0xff, 0xff, 0xfe, 0x7d, 0xff, 0xff, 0xff,
    0xfc, 0xdf, 0xff, 0xff, 0xff, 0xe0,

    /* U+0039 "9" */
    0x7f, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff,
    0xfc, 0xe7, 0xff, 0xff, 0xff, 0xef, 0xcf, 0xff,
    0xff, 0xfe, 0xff, 0x9f, 0xff, 0xff, 0xe7, 0xfe,
    0x0, 0x0, 0x0, 0x7f, 0xf0, 0x0, 0x0, 0x3,
    0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc, 0x0, 0x0,
    0x1, 0xff, 0xe0, 0x0, 0x0, 0xf, 0xff, 0x0,
    0x0, 0x0, 0x7f, 0xf8, 0x0, 0x0, 0x3, 0xff,
    0xc0, 0x0, 0x0, 0x1f, 0xfe, 0x0, 0x0, 0x0,
    0xff, 0xf0, 0x0, 0x0, 0x7, 0xff, 0x80, 0x0,
    0x0, 0x3f, 0xfc, 0x0, 0x0, 0x1, 0xff, 0xe0,
    0x0, 0x0, 0xf, 0xff, 0x0, 0x0, 0x0, 0x7f,
    0xf8, 0x0, 0x0, 0x3, 0xff, 0xc0, 0x0, 0x0,
    0x1f, 0xfe, 0x0, 0x0, 0x0, 0xff, 0xf0, 0x0,
    0x0, 0x7, 0xff, 0x80, 0x0, 0x0, 0x3f, 0xfc,
    0x0, 0x0, 0x1, 0xff, 0xe0, 0x0, 0x0, 0xf,
    0xff, 0x0, 0x0, 0x0, 0x1f, 0xfb, 0xff, 0xff,
    0xfe, 0x3f, 0xbf, 0xff, 0xff, 0xf4, 0x39, 0xff,
    0xff, 0xff, 0x38, 0x5f, 0xff, 0xff, 0xfb, 0xe0,
    0xff, 0xff, 0xff, 0xbf, 0x0, 0x0, 0x0, 0x1,
    0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0, 0x0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x0, 0x3, 0xf0, 0x0,
    0x0, 0x0, 0x1f, 0x80, 0x0, 0x0, 0x0, 0xfc,
    0x0, 0x0, 0x0, 0x7, 0xe0, 0x0, 0x0, 0x0,
    0x3f, 0x0, 0x0, 0x0, 0x1, 0xf8, 0x0, 0x0,
    0x0, 0xf, 0xc0, 0x0, 0x0, 0x0, 0x7e, 0x0,
    0x0, 0x0, 0x3, 0xf0, 0x0, 0x0, 0x0, 0x1f,
    0x80, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x0, 0x0,
    0x7, 0xe0, 0x0, 0x0, 0x0, 0x3f, 0x0, 0x0,
    0x0, 0x1, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xc0,
    0x0, 0x0, 0x0, 0x7e, 0x0, 0x0, 0x0, 0x3,
    0xf0, 0x0, 0x0, 0x0, 0x1f, 0x80, 0x0, 0x0,
    0x0, 0xfc, 0x0, 0x0, 0x0, 0x7, 0xe0, 0x3f,
    0xff, 0xff, 0x9f, 0x3, 0xff, 0xff, 0xff, 0x38,
    0x1f, 0xff, 0xff, 0xfe, 0x41, 0xff, 0xff, 0xff,
    0xfc, 0x1f, 0xff, 0xff, 0xff, 0xe0,

    /* U+003A ":" */
    0x38, 0xfb, 0xff, 0xff, 0xef, 0x8e, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x71, 0xf7,
    0xff, 0xff, 0xdf, 0x1c
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 192, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 783, .box_w = 29, .box_h = 5, .ofs_x = 10, .ofs_y = 27},
    {.bitmap_index = 20, .adv_w = 0, .box_w = 7, .box_h = 7, .ofs_x = -4, .ofs_y = 0},
    {.bitmap_index = 27, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 305, .adv_w = 783, .box_w = 6, .box_h = 58, .ofs_x = 37, .ofs_y = 2},
    {.bitmap_index = 349, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 627, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 905, .adv_w = 783, .box_w = 37, .box_h = 57, .ofs_x = 6, .ofs_y = 2},
    {.bitmap_index = 1169, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 1447, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 1725, .adv_w = 783, .box_w = 37, .box_h = 57, .ofs_x = 6, .ofs_y = 3},
    {.bitmap_index = 1989, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 2267, .adv_w = 783, .box_w = 37, .box_h = 60, .ofs_x = 6, .ofs_y = 0},
    {.bitmap_index = 2545, .adv_w = 192, .box_w = 7, .box_h = 32, .ofs_x = 2, .ofs_y = 13}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0xd, 0xe
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 15, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 3, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    },
    {
        .range_start = 48, .range_length = 11, .glyph_id_start = 4,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t dseg60 = {
#else
lv_font_t dseg60 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 60,          /*The maximum line height required by the font*/
    .base_line = 0,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -7,
    .underline_thickness = 3,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};



#endif /*#if DSEG60*/

