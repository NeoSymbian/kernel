// Pixel 3a XL (bonito, SDC panel) continuous-splash framebuffer, from the DTB's
// /chosen/framebuffer@9c000000 node: 1080x2160, stride 4320, a8r8g8b8.
#define FB_BASE   0x9c000000
#define FB_BYTES  (4320 * 2160)
#define COLOR_A64 0xff00ff00	// green
#define COLOR_A32 0xff0000ff	// blue
