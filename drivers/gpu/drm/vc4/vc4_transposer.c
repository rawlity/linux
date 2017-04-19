/*
 * Copyright © 2017 Broadcom
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/* Boris: TXP's interrupt is the next one after V3D.  Base address is
 * 0x7e004000.  You set up HVS channel to in one-shot mode, set up
 * DIM, DSTPTR, WIDTH.  Set CTRL with GO set, then wait for EI.
 *
 * Please set yourself as git commit --author, of course.
 */

/* Base address of the output.  Raster formats must be 4-byte aligned,
 * T and LT must be 16-byte aligned or maybe utile-aligned (docs are
 * inconsistent, but probably utile).
 */
#define TXP_DST_PTR		0x00

/* Pitch in bytes for raster images, 16-byte aligned.  For tiled, it's
 * the width in tiles.
 */
#define TXP_DST_PITCH		0x04
/* For T-tiled imgaes, DST_PITCH should be the number of tiles wide,
 * shifted up.
 */
# define TXP_T_TILE_WIDTH_SHIFT		7
/* For LT-tiled images, DST_PITCH should be the number of utiles wide,
 * shifted up.
 */
# define TXP_LT_TILE_WIDTH_SHIFT	4

/* Pre-rotation width/height of the image.  Must match HVS config.
 *
 * If TFORMAT and 32-bit, limit is 1920 for 32-bit and 3840 to 16-bit
 * and width/height must be tile or utile-aligned as appropriate.  If
 * transposing (rotating), width is limited to 1920.
 *
 * Height is limited to various numbers between 4088 and 4095.  I'd
 * just use 4088 to be safe.
 */
#define TXP_DIM			0x08
# define TXP_HEIGHT_SHIFT		16
# define TXP_WIDTH_SHIFT		0

#define TXP_DST_CTRL		0x0c
/* These bits are set to 0x54 */
#define TXP_PILOT_SHIFT			24
/* Bits 22-23 are set to 0x01 */
#define TXP_VERSION_SHIFT		22

/* Powers down the internal memory. */
# define TXP_POWERDOWN			BIT(21)

/* Enables storing the alpha component in 8888/4444, instead of
 * filling with ~ALPHA_INVERT.
 */
# define TXP_ALPHA_ENABLE		BIT(20)

/* 4 bits, each enables stores for a channel in each set of 4 bytes.
 * Set to 0xf for normal operation.
 */
# define TXP_BYTE_ENABLE_SHIFT		16

/* Debug: Generate VSTART again at EOF. */
# define TXP_VSTART_AT_EOF		BIT(15)

/* Debug: Terminate the current frame immediately.  Stops AXI
 * writes.
 */
# define TXP_ABORT			BIT(14)

# define TXP_DITHER			BIT(13)

/* Inverts alpha if TXP_ALPHA_ENABLE, chooses fill value for
 * !TXP_ALPHA_ENABLE.
 */
# define TXP_ALPHA_INVERT		BIT(12)

/* Note: I've listed the channels here in high bit (in byte 3/2/1) to
 * low bit (in byte 0) order.
 */
# define TXP_FORMAT_SHIFT		8
# define TXP_FORMAT_ABGR4444		0
# define TXP_FORMAT_ARGB4444		1
# define TXP_FORMAT_BGRA4444		2
# define TXP_FORMAT_RGBA4444		3
# define TXP_FORMAT_BGR565		6
# define TXP_FORMAT_RGB565		7
/* 888s are non-rotated, raster-only */
# define TXP_FORMAT_BGR888		8
# define TXP_FORMAT_RGB888		9
# define TXP_FORMAT_ABGR8888		12
# define TXP_FORMAT_ARGB8888		13
# define TXP_FORMAT_BGRA8888		14
# define TXP_FORMAT_RGBA8888		15

/* If TFORMAT is set, generates LT instead of T format. */
# define TXP_LINEAR_UTILE		BIT(7)

/* Rotate output by 90 degrees. */
# define TXP_TRANSPOSE			BIT(6)

/* Generate a tiled format for V3D. */
# define TXP_TFORMAT			BIT(5)

/* Generates some undefined test mode output. */
# define TXP_TEST_MODE			BIT(4)

/* Request odd field from HVS. */
# define TXP_FIELD			BIT(3)

/* Raise interrupt when idle. */
# define TXP_EI				BIT(2)

/* Set when generating a frame, clears when idle. */
# define TXP_BUSY			BIT(1)

/* Starts a frame.  Self-clearing. */
# define TXP_GO				BIT(0)

/* Number of lines received and committed to memory. */
#define TXP_PROGRESS		0x10
