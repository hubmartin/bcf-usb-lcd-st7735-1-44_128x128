

/*

BigClown

This is an example code how to run ST7735 controller for the color LCDs.
In this code I used 1.44" 128*128 pixel displays. Both of them are connected
in paralel.

Because the STM32L083CZ has only 192kB of flash, the graphics had to be used
in a smaller bitdepth. Also the automatic functions of moving eyes and blinking
could not fit to the flash. The solution could be improve the way the graphics
and lookup tables are stored.

Graphics had to be ported to 8 bit per pixel bitdepth. The format is R3G3B2
The python converter has been edited and the C code have byteTo565 function
to convert it back to 16bit depth.

*/

//--------------------------------------------------------------------------
// Uncanny eyes for Adafruit 1.5" OLED (product #1431) or 1.44" TFT LCD
// (#2088).  Works on PJRC Teensy 3.x and on Adafruit M0 and M4 boards
// (Feather, Metro, etc.).  This code uses features specific to these
// boards and WILL NOT work on normal Arduino or other boards!
//
// SEE FILE "config.h" FOR MOST CONFIGURATION (graphics, pins, display type,
// etc).  Probably won't need to edit THIS file unless you're doing some
// extremely custom modifications.
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing products
// from Adafruit!
//
// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.  SPI FIFO insight from Paul Stoffregen's ILI9341_t3 library.
// Inspired by David Boccabella's (Marcwolf) hybrid servo/OLED eye concept.
//--------------------------------------------------------------------------

/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#include <application.h>
#define SYMMETRICAL_EYELID
#include "hubEye.h"
//#include "defaultEye.h"

// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

bc_gfx_t gfx;

bc_lis2dh12_t a;
bc_lis2dh12_result_g_t a_result;

bc_opt3001_t lux;
float illuminance;

//extern const bc_image_t eye;

#define SENSOR_DATA_STREAM_SAMPLES 6

BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_acc_x, SENSOR_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_acc_x;

BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_acc_y, SENSOR_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_acc_y;

/*
DC
Command = 0
Data = 1
*/

#define LCD_CS BC_GPIO_P15
#define LCD_DC BC_GPIO_P17
#define LCD_RES BC_GPIO_P16


void display_init(const uint8_t *addr);
void lcd_send_command(uint8_t command);
void lcd_send_command_data(uint8_t command, const uint8_t *data, size_t data_length);
void lcd_set_pixel(uint8_t x, uint8_t y, uint16_t color);

bc_lis2dh12_t a;

// EYE ANIMATION -----------------------------------------------------------

const uint8_t ease[] = { // Ease in/out curve for eye movements 3*t^2-2*t^3
    0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,   // T
    3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,   // h
   11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,   // x
   24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,   // 2
   40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,   // A
   60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,   // l
   81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98,100,101,103,   // e
  104,106,107,109,110,112,113,115,116,118,119,121,122,124,125,127,   // c
  128,130,131,133,134,136,137,139,140,142,143,145,146,148,149,151,   // J
  152,154,155,157,158,159,161,162,164,165,167,168,170,171,172,174,   // a
  175,177,178,179,181,182,183,185,186,188,189,190,192,193,194,195,   // c
  197,198,199,201,202,203,204,205,207,208,209,210,211,213,214,215,   // o
  216,217,218,219,220,221,222,224,225,226,227,228,228,229,230,231,   // b
  232,233,234,235,236,237,237,238,239,240,240,241,242,243,243,244,   // s
  245,245,246,246,247,248,248,249,249,250,250,251,251,251,252,252,   // o
  252,253,253,253,254,254,254,254,254,255,255,255,255,255,255,255 }; // n


void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint16_t iScale,  // Scale factor for iris (0-1023)
  uint8_t  scleraX, // First pixel X offset into sclera image
  uint8_t  scleraY, // First pixel Y offset into sclera image
  uint8_t  uT,      // Upper eyelid threshold value
  uint8_t  lT);


void frame(uint16_t iScale);

//#define AUTOBLINK

// A simple state machine is used to control eye blinks/winks:
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing
#define DEBLINK 2       // Eyelid is currently opening
typedef struct {
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  bc_tick_t duration;    // Duration of blink state (micros)
  bc_tick_t startTime;   // Time (micros) of last state change
} eyeBlink;

#define NUM_EYES 1

struct {                // One-per-eye structure
  //displayType *display; // -> OLED/TFT object
  eyeBlink     blink;   // Current blink/wink state
} eye[NUM_EYES];

typedef struct {        // Struct is defined before including config.h --
  int8_t  select;       // pin numbers for each eye's screen select line
  int8_t  wink;         // and wink button (or -1 if none) specified there,
  uint8_t rotation;     // also display rotation.
} eyeInfo_t;

eyeInfo_t eyeInfo[] = {
  {  9, 0, 0 }, // LEFT EYE display-select and wink pins, no rotation
  { 10, 0, 0 }, // RIGHT EYE display-select and wink pins, no rotation
};

#define LCD_COLSTART 2
#define LCD_ROWSTART 3

#define ST_CMD_DELAY      0x80    // special signifier for command lists

#define ST77XX_NOP        0x00
#define ST77XX_SWRESET    0x01
#define ST77XX_RDDID      0x04
#define ST77XX_RDDST      0x09

#define ST77XX_SLPIN      0x10
#define ST77XX_SLPOUT     0x11
#define ST77XX_PTLON      0x12
#define ST77XX_NORON      0x13

#define ST77XX_INVOFF     0x20
#define ST77XX_INVON      0x21
#define ST77XX_DISPOFF    0x28
#define ST77XX_DISPON     0x29
#define ST77XX_CASET      0x2A
#define ST77XX_RASET      0x2B
#define ST77XX_RAMWR      0x2C
#define ST77XX_RAMRD      0x2E

#define ST77XX_PTLAR      0x30
#define ST77XX_COLMOD     0x3A
#define ST77XX_MADCTL     0x36

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1      0xDA
#define ST77XX_RDID2      0xDB
#define ST77XX_RDID3      0xDC
#define ST77XX_RDID4      0xDD


// Some register settings
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_FRMCTR1    0xB1
#define ST7735_FRMCTR2    0xB2
#define ST7735_FRMCTR3    0xB3
#define ST7735_INVCTR     0xB4
#define ST7735_DISSET5    0xB6

#define ST7735_PWCTR1     0xC0
#define ST7735_PWCTR2     0xC1
#define ST7735_PWCTR3     0xC2
#define ST7735_PWCTR4     0xC3
#define ST7735_PWCTR5     0xC4
#define ST7735_VMCTR1     0xC5

#define ST7735_PWCTR6     0xFC

#define ST7735_GMCTRP1    0xE0
#define ST7735_GMCTRN1    0xE1

// Some ready-made 16-bit ('565') color settings:
#define	ST77XX_BLACK      0x0000
#define ST77XX_WHITE      0xFFFF
#define	ST77XX_RED        0xF800
#define	ST77XX_GREEN      0x07E0
#define	ST77XX_BLUE       0x001F
#define ST77XX_CYAN       0x07FF
#define ST77XX_MAGENTA    0xF81F
#define ST77XX_YELLOW     0xFFE0
#define	ST77XX_ORANGE     0xFC00

static const uint8_t Rcmd1[] =
{                       // 7735R init, part 1 (red or green tab)
    15,                             // 15 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST77XX_INVOFF,  0,              // 13: Don't invert display, no args
    ST77XX_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      //0xC8,                         //     row/col addr, bottom-top refresh
      ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST7735_MADCTL_BGR,
    ST77XX_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05 };                       //     16-bit color

static const uint8_t Rcmd2green144[] =
{               // 7735R init, part 2 (green 1.44 tab)
    2,                              //  2 commands in list:
    ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F };                //     XEND = 127


static const uint8_t Rcmd3[] =
{                       // 7735R init, part 3 (red or green tab)
    4,                              //  4 commands in list:
    ST7735_GMCTRP1, 16      ,       //  1: Gamma setting, I think?
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: More gamma?
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST77XX_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay


void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_PRESS)
    {
        bc_led_set_mode(&led, BC_LED_MODE_TOGGLE);
    }

    // Logging in action
    bc_log_info("Button event handler - event: %i", event);
}

void lcd_send_color(uint16_t color, size_t num_pixels)
{
    //bc_spi_transfer(data, NULL, data_length);
    uint8_t dummy_read;
    (void) dummy_read;

    uint8_t r[2];
    r[0] = color >> 8;
    r[1] = color & 0xFF;

    // Set CS to active level
    GPIOB->BSRR = GPIO_BSRR_BR_12;

    for (size_t p = 0; p < num_pixels; p++)
    {
        for (int i = 0; i < 2; i++)
        {
            // Wait until transmit buffer is empty...
            while ((SPI2->SR & SPI_SR_TXE) == 0)
            {
                continue;
            }

            // Write data register
            SPI2->DR = r[i];

            // Until receive buffer is empty...
            while ((SPI2->SR & SPI_SR_RXNE) == 0)
            {
                continue;
            }

            // Read data register
            dummy_read = SPI2->DR;
        }
    }

    // Set CS to inactive level
    GPIOB->BSRR = GPIO_BSRR_BS_12;
}

void lcd_send_data(const uint8_t *data, size_t data_length)
{
    //bc_spi_transfer(data, NULL, data_length);
    uint8_t dummy_read;
    (void) dummy_read;

    if (data_length == 0)
    {
        return;
    }

    // Set CS to active level
    GPIOB->BSRR = GPIO_BSRR_BR_12;

    while (true)
    {

        if (data_length == 0)
        {
            break;
        }

        // Wait until transmit buffer is empty...
        while ((SPI2->SR & SPI_SR_TXE) == 0)
        {
            continue;
        }

        // Write data register
        SPI2->DR = *data++;

        // Until receive buffer is empty...
        while ((SPI2->SR & SPI_SR_RXNE) == 0)
        {
            continue;
        }

        // Read data register
        dummy_read = SPI2->DR;

        data_length--;
    }

    while ((SPI2->SR & SPI_SR_BSY) == 1)
    {
        continue;
    }
/*
    bc_timer_init();
    bc_timer_start();
    bc_timer_delay(10);
    bc_timer_stop();*/

    // Set CS to inactive level
    GPIOB->BSRR = GPIO_BSRR_BS_12;
}

void lcd_send_command(uint8_t command)
{
    // Command mode
    bc_gpio_set_output(LCD_DC, 0);

    lcd_send_data(&command, 1);

    // Data mode
    bc_gpio_set_output(LCD_DC, 1);

}

void lcd_send_command_data(uint8_t command, const uint8_t *data, size_t data_length)
{
    lcd_send_command(command);

    lcd_send_data(data, data_length);
}

void lcd_set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t data[4];

    data[0] = 0x00;
    data[1] = x0 + LCD_COLSTART;
    data[2] = 0x00;
    data[3] = x1 + LCD_COLSTART;

    lcd_send_command_data(ST77XX_CASET, data, sizeof(data));

    data[0] = 0x00;
    data[1] = y0 + LCD_ROWSTART;
    data[2] = 0x00;
    data[3] = y1 + LCD_ROWSTART;

    lcd_send_command_data(ST77XX_RASET, data, sizeof(data));
}

void lcd_set_pixel(uint8_t x, uint8_t y, uint16_t color)
{
    lcd_set_addr_window(x, y, x, y);

    // Ram write
    lcd_send_command(ST77XX_RAMWR);
    lcd_send_color(color, 1);
}

void lcd_clear(uint16_t color)
{
    lcd_set_addr_window(0, 0, 128-1, 128-1);

    // Ram write
    lcd_send_command(ST77XX_RAMWR);
    lcd_send_color(color, 128*128);
}

bc_dma_channel_config_t _bc_spi_dma_config =
{
    .request = BC_DMA_REQUEST_2,
    .direction = BC_DMA_DIRECTION_TO_PERIPHERAL,
    .data_size_memory = BC_DMA_SIZE_1,
    .data_size_peripheral = BC_DMA_SIZE_1,
    .mode = BC_DMA_MODE_STANDARD,
    .address_peripheral = (void *)&SPI2->DR,
    .priority = BC_DMA_PRIORITY_HIGH
};

void lcd_send_dma(uint8_t *data, size_t data_size)
{

    // Disable SPI2
    SPI2->CR1 &= ~SPI_CR1_SPE;

    // Enable TX DMA request
    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    // Enable SPI2
    SPI2->CR1 |= SPI_CR1_SPE;

    // Set CS to active level
    GPIOB->BSRR = GPIO_BSRR_BR_12;

    // Setup DMA channel
    _bc_spi_dma_config.address_memory = (void *)data;
    _bc_spi_dma_config.length = data_size;
    bc_dma_channel_config(BC_DMA_CHANNEL_5, &_bc_spi_dma_config);
    bc_dma_channel_run(BC_DMA_CHANNEL_5);
}

void lcd_send_dma_wait()
{
    while(DMA1_Channel5->CCR & DMA_CCR_EN) {}

    // Set CS to inactive level
    GPIOB->BSRR = GPIO_BSRR_BS_12;

    // Disable SPI2
    SPI2->CR1 &= ~SPI_CR1_SPE;

    // Disable TX DMA request
    SPI2->CR2 &= ~SPI_CR2_TXDMAEN;

    // Enable SPI2
    SPI2->CR1 |= SPI_CR1_SPE;
}

void lcd_draw_image(int x, int y, const bc_image_t *img)
{
    lcd_set_addr_window(x, y, x + img->width -1, y + img->height -1);
    //lcd_push_color(color);

    // Ram write
    lcd_send_command(ST77XX_RAMWR);

    lcd_send_data(img->data, img->width * img->height * 2);
    /*
    // Set CS to active level
    GPIOB->BSRR = GPIO_BSRR_BR_12;

    //bc_spi_async_transfer(img->data, NULL, img->width * img->height * 2, NULL, NULL);

            // Disable SPI2
        SPI2->CR1 &= ~SPI_CR1_SPE;

        // Enable TX DMA request
        SPI2->CR2 |= SPI_CR2_TXDMAEN;

        // Enable SPI2
        SPI2->CR1 |= SPI_CR1_SPE;

        // Setup DMA channel
        _bc_spi_dma_config.address_memory = (void *)img->data;
        _bc_spi_dma_config.length = img->width * img->height * 2;
        bc_dma_channel_config(BC_DMA_CHANNEL_5, &_bc_spi_dma_config);
        bc_dma_channel_run(BC_DMA_CHANNEL_5);

    while (bc_dma_channel_get_length(BC_DMA_CHANNEL_5) != 0)
    {
    }

    // Set CS to active level
    GPIOB->BSRR = GPIO_BSRR_BS_12;*/

}

void bc_delay(uint32_t ms)
{
    bc_tick_t t = bc_tick_get();
    while(bc_tick_get() < (t + ms));
}

void display_init(const uint8_t *addr)
{

  uint8_t  numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = *addr++;   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    cmd = *addr++;         // Read command
    numArgs  = *addr++;    // Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    lcd_send_command_data(cmd, addr, numArgs);
    addr += numArgs;

    if(ms) {
      ms = *addr++; // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      bc_delay(ms);
    }
  }
}

bool lcd_driver_is_ready(void *self)
{
    return true;
}

void lcd_driver_clear(void *self)
{
    lcd_clear(0xFFFF);
}

void lcd_driver_draw_pixel(void *self, int x, int y, uint32_t color)
{
    lcd_set_pixel(x, y, color);
}

uint32_t lcd_driver_get_pixel(void *self, int x, int y)
{
    return 0;
}

bool lcd_driver_update(void *self, int x, int y)
{
    return true;
}

bc_gfx_caps_t lcd_driver_get_caps(void *self)
{
    bc_gfx_caps_t caps = { .width = 128, .height = 128 };

    return caps;
}

const bc_gfx_driver_t *lcd_get_driver(void)
{
    static const bc_gfx_driver_t driver =
    {
        .is_ready = (bool (*)(void *)) lcd_driver_is_ready,
        .clear = (void (*)(void *)) lcd_driver_clear,
        .draw_pixel = (void (*)(void *, int, int, uint32_t)) lcd_driver_draw_pixel,
        .get_pixel = (uint32_t (*)(void *, int, int)) lcd_driver_get_pixel,
        .update = (bool (*)(void *)) lcd_driver_update,
        .get_caps = (bc_gfx_caps_t (*)(void *)) lcd_driver_get_caps
    };

    return &driver;
}

void lcd_init()
{
    bc_system_pll_enable();

    //bc_spi_init(BC_SPI_SPEED_4_MHZ, BC_SPI_MODE_0);
    bc_spi_init(BC_SPI_SPEED_16_MHZ, BC_SPI_MODE_0);

    // Debug GPIO
    bc_gpio_init(BC_GPIO_P8);
    bc_gpio_set_mode(BC_GPIO_P8, BC_GPIO_MODE_OUTPUT);

    bc_gpio_set_mode(LCD_DC, BC_GPIO_MODE_OUTPUT);
    // Data mode
    bc_gpio_set_output(LCD_DC, 1);

    bc_gpio_set_mode(LCD_RES, BC_GPIO_MODE_OUTPUT);
    // RESET
    bc_gpio_set_output(LCD_RES, 0);
    bc_delay(100);
    bc_gpio_set_output(LCD_RES, 1);
    bc_delay(100);

    display_init(Rcmd1);
    display_init(Rcmd2green144);
    display_init(Rcmd3);

}

float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
    int32_t val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (val > out_max)
    {
        val = out_max;
    }
    if (val < out_min)
    {
        val = out_min;
    }

    return val;
}

void lis2_event_handler(bc_lis2dh12_t *self, bc_lis2dh12_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == BC_LIS2DH12_EVENT_UPDATE) {
        bc_lis2dh12_get_result_g(&a, &a_result);

        if (bc_lis2dh12_get_result_g(&a, &a_result))
        {
            bc_data_stream_feed(&stream_acc_x, &a_result.x_axis);
            bc_data_stream_feed(&stream_acc_y, &a_result.y_axis);
        }
        else
        {
            bc_data_stream_reset(&stream_acc_x);
            bc_data_stream_reset(&stream_acc_y);
        }
    }
}

void lux_meter_event_handler(bc_tag_lux_meter_t *self, bc_tag_lux_meter_event_t event, void *event_param)
{
    (void) event_param;

    if (event == BC_TAG_LUX_METER_EVENT_UPDATE)
    {
        bc_tag_lux_meter_get_illuminance_lux(self, &illuminance);
    }
}

void application_init(void)
{
    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_ON);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    lcd_init();
/*
    for(;;)
    {
        //lcd_clear(ST77XX_RED);
        //lcd_clear(ST77XX_GREEN);

        lcd_draw_image(0 ,0 , &eye);
        lcd_clear(ST77XX_WHITE);

    }*/

    bc_gfx_init(&gfx, NULL, lcd_get_driver());
/*
    lcd_draw_image(0 ,0 , &eye);

    bc_gfx_draw_pixel(&gfx, 0,0, ST77XX_RED);
    bc_gfx_draw_pixel(&gfx, 1,0, ST77XX_GREEN);
    bc_gfx_draw_pixel(&gfx, 2,0, ST77XX_BLUE);

    bc_gfx_set_font(&gfx, &bc_font_ubuntu_13);
    bc_gfx_draw_string(&gfx, 5, 5, "BigClown", ST77XX_RED);
    bc_gfx_draw_string(&gfx, 5, 20, "ST7735 LCD Driver", ST77XX_GREEN);
    bc_gfx_draw_string(&gfx, 5, 35, "128x128 px RGB", ST77XX_BLUE);
*/
    bc_lis2dh12_init(&a, BC_I2C_I2C0, 0x19);
    bc_lis2dh12_set_event_handler(&a, lis2_event_handler, NULL);
    bc_lis2dh12_set_update_interval(&a, 50);

    bc_tag_lux_meter_init(&lux, BC_I2C_I2C0, BC_TAG_LUX_METER_I2C_ADDRESS_DEFAULT);
    bc_tag_lux_meter_set_event_handler(&lux, lux_meter_event_handler, NULL);
    bc_tag_lux_meter_set_update_interval(&lux, 200);

    bc_data_stream_init(&stream_acc_x, 1, &stream_buffer_acc_x);
    bc_data_stream_init(&stream_acc_y, 1, &stream_buffer_acc_y);


}

void manual_eye_control()
{
    bc_lis2dh12_result_g_t acc_avg;

    bc_data_stream_get_average(&stream_acc_x, &acc_avg.x_axis);
    bc_data_stream_get_average(&stream_acc_y, &acc_avg.y_axis);

    uint8_t scleraX = map_f(acc_avg.x_axis, -0.5f, +0.5f, 0, 65);
    uint8_t scleraY = map_f(acc_avg.y_axis, -0.5f, +0.5f, 0, 65);

    uint8_t irisScale = map_f(illuminance, 15000, 0, 2, 255);

    drawEye(0, irisScale, scleraX, scleraY, 100, 100);
}

void application_task(void)
{
    manual_eye_control();

    //frame(120);

    // Plan next run this function after 10 ms
    bc_scheduler_plan_current_from_now(10);
}


inline uint16_t byteTo565(uint8_t color)
{
    return  (color & 0xE0) << 8 |
            (color & 0x1C) << 6 |
            (color & 0x03) << 3;
}


void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint16_t iScale,  // Scale factor for iris (0-1023)
  uint8_t  scleraX, // First pixel X offset into sclera image
  uint8_t  scleraY, // First pixel Y offset into sclera image
  uint8_t  uT,      // Upper eyelid threshold value
  uint8_t  lT) {    // Lower eyelid threshold value

  uint8_t  screenX, screenY, scleraXsave;
  int16_t  irisX, irisY;
  uint16_t p, a;
  uint32_t d;

  static uint16_t lineBuffer[SCREEN_WIDTH];

#if defined(SYNCPIN) && (SYNCPIN >= 0)
  if(receiver) {
    // Overwrite arguments with values in syncStruct.  Disable interrupts
    // briefly so new data can't overwrite the struct in mid-parse.
    noInterrupts();
    iScale  = syncStruct.iScale;
    // Screen is mirrored, this 'de-mirrors' the eye X direction
    scleraX = SCLERA_WIDTH - 1 - SCREEN_WIDTH - syncStruct.scleraX;
    scleraY = syncStruct.scleraY;
    uT      = syncStruct.uT;
    lT      = syncStruct.lT;
    interrupts();
  } else {
    // Stuff arguments into syncStruct and send to receiver
    syncStruct.iScale  = iScale;
    syncStruct.scleraX = scleraX;
    syncStruct.scleraY = scleraY;
    syncStruct.uT      = uT;
    syncStruct.lT      = lT;
    Wire.beginTransmission(SYNCADDR);
    Wire.write((char *)&syncStruct, sizeof syncStruct);
    Wire.endTransmission();
  }
#endif

  uint8_t  irisThreshold = (128 * (1023 - iScale) + 512) / 1024;
  uint32_t irisScale     = IRIS_MAP_HEIGHT * 65536 / irisThreshold;

  lcd_set_addr_window(0, 0, 128-1, 128-1);

    // Ram write
    lcd_send_command(ST77XX_RAMWR);

  scleraXsave = scleraX; // Save initial X value to reset on each line
  irisY       = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;
  for(screenY=0; screenY<SCREEN_HEIGHT; screenY++, scleraY++, irisY++) {

      // Debug pin
    bc_gpio_set_output(BC_GPIO_P8, 1);

    uint16_t *ptr = lineBuffer;

    scleraX = scleraXsave;
    irisX   = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;
    for(screenX=0; screenX<SCREEN_WIDTH; screenX++, scleraX++, irisX++) {
      if((lower[screenY][screenX] <= lT) ||
         (upper[screenY][screenX] <= uT)) {             // Covered by eyelid
        p = 0;
      } else if((irisY < 0) || (irisY >= IRIS_HEIGHT) ||
                (irisX < 0) || (irisX >= IRIS_WIDTH)) { // In sclera
        p = byteTo565(sclera[scleraY][scleraX]);
      } else {                                          // Maybe iris...
        p = polar[irisY][irisX];                        // Polar angle/dist
        d = p & 0x7F;                                   // Distance from edge (0-127)
        if(d < irisThreshold) {                         // Within scaled iris area
          d = d * irisScale / 65536;                    // d scaled to iris image height
          a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;        // Angle (X)
          p = byteTo565(iris[d][a]);                               // Pixel = iris
        } else {                                        // Not in iris
          p = byteTo565(sclera[scleraY][scleraX]);                 // Pixel = sclera
        }
      }

      *ptr++ = __builtin_bswap16(p); // DMA: store in scanline buffer
       // lcd_send_color(p, 1);
    } // end column

    bc_gpio_set_output(BC_GPIO_P8, 0);

    lcd_send_dma_wait();

    lcd_send_dma((uint8_t*)lineBuffer, sizeof(lineBuffer));
  } // end scanline

    lcd_send_dma_wait();
}






#ifdef AUTOBLINK
bc_tick_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
#endif

void frame( // Process motion for a single frame of left or right eye
  uint16_t        iScale) {     // Iris scale (0-1023) passed in
  static uint32_t frames   = 0; // Used in frame rate calculation
  static uint8_t  eyeIndex = 0; // eye[] array counter
  int16_t         eyeX, eyeY;
  bc_tick_t        t = bc_tick_get(); // Time at start of function
/*
  if(!(++frames & 255)) { // Every 256 frames...
    uint32_t elapsed = (millis() - startTime) / 1000;
    if(elapsed) Serial.println(frames / elapsed); // Print FPS
  }*/

  if(++eyeIndex >= NUM_EYES) eyeIndex = 0; // Cycle through eyes, 1 per call

  // X/Y movement

#if defined(JOYSTICK_X_PIN) && (JOYSTICK_X_PIN >= 0) && \
    defined(JOYSTICK_Y_PIN) && (JOYSTICK_Y_PIN >= 0)

  // Read X/Y from joystick, constrain to circle
  int16_t dx, dy;
  int32_t d;
  eyeX = analogRead(JOYSTICK_X_PIN); // Raw (unclipped) X/Y reading
  eyeY = analogRead(JOYSTICK_Y_PIN);
#ifdef JOYSTICK_X_FLIP
  eyeX = 1023 - eyeX;
#endif
#ifdef JOYSTICK_Y_FLIP
  eyeY = 1023 - eyeY;
#endif
  dx = (eyeX * 2) - 1023; // A/D exact center is at 511.5.  Scale coords
  dy = (eyeY * 2) - 1023; // X2 so range is -1023 to +1023 w/center at 0.
  if((d = (dx * dx + dy * dy)) > (1023 * 1023)) { // Outside circle
    d    = (int32_t)sqrt((float)d);               // Distance from center
    eyeX = ((dx * 1023 / d) + 1023) / 2;          // Clip to circle edge,
    eyeY = ((dy * 1023 / d) + 1023) / 2;          // scale back to 0-1023
  }

#else // Autonomous X/Y eye motion
      // Periodically initiates motion to a new random point, random speed,
      // holds there for random period until next motion.

  static bool  eyeInMotion      = false;
  static int16_t  eyeOldX=512, eyeOldY=512, eyeNewX=512, eyeNewY=512;
  static uint32_t eyeMoveStartTime = 0L;
  static int32_t  eyeMoveDuration  = 0L;

  int32_t dt = t - eyeMoveStartTime;      // uS elapsed since last eye event
  if(eyeInMotion) {                       // Currently moving?
    if(dt >= eyeMoveDuration) {           // Time up?  Destination reached.
      eyeInMotion      = false;           // Stop moving
      eyeMoveDuration  = rand() % 3000; //TODOrandom(3000000); // 0-3 sec stop
      eyeMoveStartTime = t;               // Save initial time of stop
      eyeX = eyeOldX = eyeNewX;           // Save position
      eyeY = eyeOldY = eyeNewY;
    } else { // Move time's not yet fully elapsed -- interpolate position
      int16_t e = ease[255 * dt / eyeMoveDuration] + 1;   // Ease curve
      eyeX = eyeOldX + (((eyeNewX - eyeOldX) * e) / 256); // Interp X
      eyeY = eyeOldY + (((eyeNewY - eyeOldY) * e) / 256); // and Y
    }
  } else {                                // Eye stopped
    eyeX = eyeOldX;
    eyeY = eyeOldY;
    if(dt > eyeMoveDuration) {            // Time up?  Begin new move.
      int16_t  dx, dy;
      uint32_t d;
      do {                                // Pick new dest in circle
        eyeNewX = rand() % 1024; //TODOrandom(1024);
        eyeNewY = rand() % 1024;//TODOrandom(1024);
        dx      = (eyeNewX * 2) - 1023;
        dy      = (eyeNewY * 2) - 1023;
      } while((d = (dx * dx + dy * dy)) > (1023 * 1023)); // Keep trying
      eyeMoveDuration  = 500; //TODOrandom(72000, 144000); // ~1/14 - ~1/7 sec
      eyeMoveStartTime = t;               // Save initial time of move
      eyeInMotion      = true;            // Start move on next frame
    }
  }

#endif // JOYSTICK_X_PIN etc.

  // Blinking

#ifdef AUTOBLINK
  // Similar to the autonomous eye movement above -- blink start times
  // and durations are random (within ranges).
  if((t - timeOfLastBlink) >= timeToNextBlink) { // Start new blink?
    timeOfLastBlink = t;
    uint32_t blinkDuration = random(36000, 72000); // ~1/28 - ~1/14 sec
    // Set up durations for both eyes (if not already winking)
    for(uint8_t e=0; e<NUM_EYES; e++) {
      if(eye[e].blink.state == NOBLINK) {
        eye[e].blink.state     = ENBLINK;
        eye[e].blink.startTime = t;
        eye[e].blink.duration  = blinkDuration;
      }
    }
    timeToNextBlink = blinkDuration * 3 + (rand() % 4000); //TODOrandom(4000000);
  }
#endif

  if(eye[eyeIndex].blink.state) { // Eye currently blinking?
    // Check if current blink state time has elapsed
    if((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) {
      // Yes -- increment blink state, unless...
      if((eye[eyeIndex].blink.state == ENBLINK) && ( // Enblinking and...
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
        (digitalRead(BLINK_PIN) == LOW) ||           // blink or wink held...
#endif
        ((eyeInfo[eyeIndex].wink >= 0) /*&&
         digitalRead(eyeInfo[eyeIndex].wink) == LOW*/) )) {
        // Don't advance state yet -- eye is held closed instead
      } else { // No buttons, or other state...
        if(++eye[eyeIndex].blink.state > DEBLINK) { // Deblinking finished?
          eye[eyeIndex].blink.state = NOBLINK;      // No longer blinking
        } else { // Advancing from ENBLINK to DEBLINK mode
          eye[eyeIndex].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
          eye[eyeIndex].blink.startTime = t;
        }
      }
    }
  } else { // Not currently blinking...check buttons!
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
    if(digitalRead(BLINK_PIN) == LOW) {
      // Manually-initiated blinks have random durations like auto-blink
      uint32_t blinkDuration = random(36000, 72000);
      for(uint8_t e=0; e<NUM_EYES; e++) {
        if(eye[e].blink.state == NOBLINK) {
          eye[e].blink.state     = ENBLINK;
          eye[e].blink.startTime = t;
          eye[e].blink.duration  = blinkDuration;
        }
      }
    } else
#endif
    if((eyeInfo[eyeIndex].wink >= 0) /*&&
       (digitalRead(eyeInfo[eyeIndex].wink) == LOW)*/) { // Wink!
      eye[eyeIndex].blink.state     = ENBLINK;
      eye[eyeIndex].blink.startTime = t;
      eye[eyeIndex].blink.duration  = random(45000, 90000);
    }
  }

  // Process motion, blinking and iris scale into renderable values

  // Scale eye X/Y positions (0-1023) to pixel units used by drawEye()
  eyeX = map_f(eyeX, 0, 1023, 0, SCLERA_WIDTH  - 128);
  eyeY = map_f(eyeY, 0, 1023, 0, SCLERA_HEIGHT - 128);
  if(eyeIndex == 1) eyeX = (SCLERA_WIDTH - 128) - eyeX; // Mirrored display

  // Horizontal position is offset so that eyes are very slightly crossed
  // to appear fixated (converged) at a conversational distance.  Number
  // here was extracted from my posterior and not mathematically based.
  // I suppose one could get all clever with a range sensor, but for now...
  if(NUM_EYES > 1) eyeX += 4;
  if(eyeX > (SCLERA_WIDTH - 128)) eyeX = (SCLERA_WIDTH - 128);

  // Eyelids are rendered using a brightness threshold image.  This same
  // map can be used to simplify another problem: making the upper eyelid
  // track the pupil (eyes tend to open only as much as needed -- e.g. look
  // down and the upper eyelid drops).  Just sample a point in the upper
  // lid map slightly above the pupil to determine the rendering threshold.
  static uint8_t uThreshold = 128;
  uint8_t        lThreshold, n;
#ifdef TRACKING
  int16_t sampleX = SCLERA_WIDTH  / 2 - (eyeX / 2), // Reduce X influence
          sampleY = SCLERA_HEIGHT / 2 - (eyeY + IRIS_HEIGHT / 4);
  // Eyelid is slightly asymmetrical, so two readings are taken, averaged
  if(sampleY < 0) n = 0;
  else            n = (upper[sampleY][sampleX] +
                       upper[sampleY][SCREEN_WIDTH - 1 - sampleX]) / 2;
  uThreshold = (uThreshold * 3 + n) / 4; // Filter/soften motion
  // Lower eyelid doesn't track the same way, but seems to be pulled upward
  // by tension from the upper lid.
  lThreshold = 254 - uThreshold;
#else // No tracking -- eyelids full open unless blink modifies them
  uThreshold = lThreshold = 0;
#endif

  // The upper/lower thresholds are then scaled relative to the current
  // blink position so that blinks work together with pupil tracking.
  if(eye[eyeIndex].blink.state) { // Eye currently blinking?
    uint32_t s = (t - eye[eyeIndex].blink.startTime);
    if(s >= eye[eyeIndex].blink.duration) s = 255;   // At or past blink end
    else s = 255 * s / eye[eyeIndex].blink.duration; // Mid-blink
    s          = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
    n          = (uThreshold * s + 254 * (257 - s)) / 256;
    lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
  } else {
    n          = uThreshold;
  }

  // Pass all the derived values to the eye-rendering function:
  drawEye(eyeIndex, iScale, eyeX, eyeY, n, lThreshold);
}
