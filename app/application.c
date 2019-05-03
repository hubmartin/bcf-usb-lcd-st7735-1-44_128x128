#include <application.h>


// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

bc_gfx_t gfx;

extern const bc_image_t eye;


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

static bc_dma_channel_config_t _bc_spi_dma_config =
{
    .request = BC_DMA_REQUEST_2,
    .direction = BC_DMA_DIRECTION_TO_PERIPHERAL,
    .data_size_memory = BC_DMA_SIZE_1,
    .data_size_peripheral = BC_DMA_SIZE_1,
    .mode = BC_DMA_MODE_STANDARD,
    .address_peripheral = (void *)&SPI2->DR,
    .priority = BC_DMA_PRIORITY_HIGH
};

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

    bc_spi_init(BC_SPI_SPEED_16_MHZ, BC_SPI_MODE_0);

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

    lcd_draw_image(0 ,0 , &eye);

    bc_gfx_draw_pixel(&gfx, 0,0, ST77XX_RED);
    bc_gfx_draw_pixel(&gfx, 1,0, ST77XX_GREEN);
    bc_gfx_draw_pixel(&gfx, 2,0, ST77XX_BLUE);

    bc_gfx_set_font(&gfx, &bc_font_ubuntu_13);
    bc_gfx_draw_string(&gfx, 5, 5, "BigClown", ST77XX_RED);
    bc_gfx_draw_string(&gfx, 5, 20, "ST7735 LCD Driver", ST77XX_GREEN);
    bc_gfx_draw_string(&gfx, 5, 35, "128x128 px RGB", ST77XX_BLUE);

}

void application_task(void)
{


    static uint8_t x = 0;

    static uint16_t color = ST77XX_RED;

    if(x >= 128)
    {
        x = 0;
        if (color == ST77XX_RED)
        {
            color = ST77XX_GREEN;
        }
        else
        {
            color = ST77XX_RED;
        }
    }

    lcd_draw_image(0 ,0 , &eye);

    bc_gfx_printf(&gfx, x, 60, color, "%d", x);

    x++;

    // Plan next run this function after 1000 ms
    bc_scheduler_plan_current_from_now(10);
}
