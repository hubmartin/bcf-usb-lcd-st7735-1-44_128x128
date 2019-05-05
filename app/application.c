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

extern const bc_image_t eye;

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

void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint16_t iScale,  // Scale factor for iris (0-1023)
  uint8_t  scleraX, // First pixel X offset into sclera image
  uint8_t  scleraY, // First pixel Y offset into sclera image
  uint8_t  uT,      // Upper eyelid threshold value
  uint8_t  lT);

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
    bc_delay(20);
    bc_gpio_set_output(LCD_RES, 1);

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

void application_task(void)
{

    bc_lis2dh12_result_g_t acc_avg;

    bc_data_stream_get_average(&stream_acc_x, &acc_avg.x_axis);
    bc_data_stream_get_average(&stream_acc_y, &acc_avg.y_axis);

    uint8_t scleraX = map_f(acc_avg.x_axis, -0.5f, +0.5f, 0, 65);
    uint8_t scleraY = map_f(acc_avg.y_axis, -0.5f, +0.5f, 0, 65);

    uint8_t irisScale = map_f(illuminance, 15000, 0, 2, 255);

    drawEye(0, irisScale, scleraX, scleraY, 100, 100);

    // Plan next run this function after 1000 ms
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

  // Set up raw pixel dump to entire screen.  Although such writes can wrap
  // around automatically from end of rect back to beginning, the region is
  // reset on each frame here in case of an SPI glitch.
  /*
  TFT_SPI.beginTransaction(settings);
  digitalWrite(eyeInfo[e].select, LOW);                        // Chip select
#if defined(_ADAFRUIT_ST7735H_) || defined(_ADAFRUIT_ST77XXH_) // TFT
  eye[e].display->setAddrWindow(0, 0, 128, 128);
#else // OLED
  eye[e].display->writeCommand(SSD1351_CMD_SETROW);    // Y range
  eye[e].display->spiWrite(0); eye[e].display->spiWrite(SCREEN_HEIGHT - 1);
  eye[e].display->writeCommand(SSD1351_CMD_SETCOLUMN); // X range
  eye[e].display->spiWrite(0); eye[e].display->spiWrite(SCREEN_WIDTH  - 1);
  eye[e].display->writeCommand(SSD1351_CMD_WRITERAM);  // Begin write
#endif
  digitalWrite(eyeInfo[e].select, LOW);                // Re-chip-select
  digitalWrite(DISPLAY_DC, HIGH);                      // Data mode
  */
  // Now just issue raw 16-bit values for every pixel...

  lcd_set_addr_window(0, 0, 128-1, 128-1);

    // Ram write
    lcd_send_command(ST77XX_RAMWR);

    // Set CS to active level
    //GPIOB->BSRR = GPIO_BSRR_BR_12;



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
