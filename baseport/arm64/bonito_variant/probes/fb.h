// Pixel 3a XL (bonito, SDC panel) continuous-splash framebuffer, from the DTB's
// /chosen/framebuffer@9c000000 node: 1080x2160, stride 4320, a8r8g8b8.
// another phone: FB_BASE=… FB_BYTES=… probes/build.sh (OnePlus 6T: 0x9d400000 0x9a3f80, docs fajita.md OP1)
#ifndef FB_BASE
#define FB_BASE   0x9c000000
#endif
#ifndef FB_BYTES
#define FB_BYTES  (4320 * 2160)
#endif
#define COLOR_A64 0xff00ff00	// green
#define COLOR_A32 0xff0000ff	// blue
