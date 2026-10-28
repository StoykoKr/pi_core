#define USER_SETUP_INFO "CYD_ST7789_Fixed"

#define ST7789_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// --- THE FIXES ---
#define TFT_INVERSION_OFF    // Fixes the "Negative" image (White instead of Black)
#define TFT_RGB_ORDER TFT_BGR // Fixes Red looking like Blue (If still wrong, change to TFT_RGB)

// --- PINOUT ---
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1
#define TFT_BL   21


#define USE_HSPI_PORT
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define SMOOTH_FONT
#define SPI_FREQUENCY  27000000 
#define SPI_TOUCH_FREQUENCY 2500000
#define TOUCH_CS 33 // Chip Select for Touch Screen



// Not even sure if we need this to be honest but right now I just record and move on