/**
 * @file display_st7735.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief st7735显示屏驱动
 * @version 0.1
 * @date 2023-05-05
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "display_st7735.h"
#include "lvgl.h"
#include "spi.h"
#include "cmsis_os2.h"

static inline void write_command(ST7735ObjectTypeDef *self, uint8_t command)
{
    self->set_dc_cs(self, 0, 0);
    self->spi_write(self, &command, 1);
    self->set_dc_cs(self, 1, 1);
}

static inline void write_data(ST7735ObjectTypeDef *self, uint8_t *data, size_t len)
{
    self->set_dc_cs(self, 1, 0);
    self->spi_write(self, data, len);
    self->set_dc_cs(self, 1, 1);
}

static void write_a_byte(ST7735ObjectTypeDef *self, uint8_t data)
{
    self->set_dc_cs(self, 1, 0);
    self->spi_write(self, &data, 1);
    self->set_dc_cs(self, 1, 1);
}

static void st7735_hard_reset(ST7735ObjectTypeDef *self)
{
    self->set_res(self, 0);
    self->sleep_ms(self, 100);
    self->set_res(self, 1);
    self->sleep_ms(self, 100);
}


void st7735_soft_reset(ST7735ObjectTypeDef *self)   // Soft reset display.
{
    write_command(self, ST7735_SWRESET);
    self->sleep_ms(self, 200);
}

void st7735_sleep_mode(ST7735ObjectTypeDef *self, bool value) //Enable(1) or disable(0) display sleep mode.
{
    if (value) {
        write_command(self, ST7735_SLPIN);
    } else {
        write_command(self, ST7735_SLPOUT);
    }
}
void st7735_inversion_mode(ST7735ObjectTypeDef *self, bool value)  //Enable(1) or disable(0) display inversion mode.
{
    if (value) {
        write_command(self, ST7735_INVON);
    } else {
        write_command(self, ST7735_INVOFF);
    }
}

void st7735_set_color_mode(ST7735ObjectTypeDef *self, uint8_t mode)  //Set display color mode.
{
    write_command(self, ST7735_COLMOD);
    write_data(self, &mode, 1);
}

void st7735_rotation(ST7735ObjectTypeDef *self, int rotation)
{
    /*Set display rotation.
        Args:
            rotation (int):
                - 0-Portrait
                - 1-Landscape
                - 2-Inverted Portrait
                - 3-Inverted Landscape*/
    write_command(self, ST7735_MADCTL);
    uint8_t byte = 0;
    switch (rotation) {
        case 0:
            byte = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB;
            break;
        case 1:
            byte = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
            break;
        case 2:
            byte = ST7735_MADCTL_RGB;
            break;
        case 3:
            byte = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
            break;
        default:
            break;
    }
    write_data(self, &byte, 1);
}

void st7735_set_columns(ST7735ObjectTypeDef *self, int start, int end)
{
    start += self->x_offset;
    end += self->x_offset;
    uint8_t data[] = {start >> 8, start & 0xFF, end >> 8, end & 0xFF};
    write_command(self, ST7735_CASET);
    write_data(self, data, 4);
}
void st7735_set_rows(ST7735ObjectTypeDef *self, int start, int end)
{
    start += self->y_offset;
    end += self->y_offset;
    uint8_t data[] = {start >> 8, start & 0xFF, end >> 8, end & 0xFF};
    write_command(self, ST7735_RASET);
    write_data(self, data, 4);
}

void st7735_set_window(ST7735ObjectTypeDef *self, int x0, int y0, int x1, int y1)
{
    st7735_set_columns(self, x0, x1);
    st7735_set_rows(self, y0, y1);
    write_command(self, ST7735_RAMWR);
}


static void export_st7735_draw_pixel(DisplayObjectTypeDef *self_base, int x, int y, uint32_t color)
{
    //设置指定像素为指定颜色
    //Args: x (int): X轴坐标, Y (int): Y轴坐标, color (int): 565格式颜色
    st7735_set_window((ST7735ObjectTypeDef*)self_base, x, y, x, y);
    uint8_t data[] = {color >> 8, (uint8_t)color};
    write_data((ST7735ObjectTypeDef*)self_base, data, 2);
}

static void export_st7735_draw_bitmap(DisplayObjectTypeDef *self_base, int x1, int y1, int x2, int y2, uint8_t *data)
{
    //复制缓冲区到屏幕的指定位置
    //Args:data (bytes): 要显示的数据
    //x (int): 缓冲区在屏幕位置的左上角X轴坐标, Y (int): 缓冲区在屏幕位置的左上角Y轴坐标
    st7735_set_window((ST7735ObjectTypeDef*)self_base, x1, y1, x2, y2);
    write_data((ST7735ObjectTypeDef*)self_base, data, 2 * ((x2 - x1) + 1) * ((y2 - y1) + 1));
}

void st7735_fill_rect(ST7735ObjectTypeDef *self, int x, int y, int width, int height, uint32_t color)
{
    //Draw a rectangle at the given location, size and filled with color.
    //Args: x (int): Top left corner x coordinate, y (int): Top left corner y coordinate
    //width (int): Width in pixels, height (int): Height in pixels
    //color (int): 565 encoded color
    st7735_set_window(self, x, y, 30, 30);

    //uint16_t pixel16[1];
    //pixel16[0] = color;
    uint8_t pixel[2];
    pixel[0] = (color >> 8);
    pixel[1] = color & 0xff;

//   div_t output;
//   output = div(width *= height, 128);
//    int chunks = output.quot;

    self->set_dc_cs(self, 1, 0); //Open to write
    size_t send_buffer = 128 * 2;
    uint8_t draw_pixel[send_buffer];

    for (int i = 0; i < send_buffer; i++) {
        draw_pixel[i] = pixel[i % 2];
    }

    for (int i = 128; i != 0; i--) {
        write_data(self, draw_pixel, send_buffer);
    }
}


void st7735_fill(ST7735ObjectTypeDef *self, uint32_t color)
{
    uint16_t i;
    st7735_set_window(self, 0, 0, self->base.width - 1, self->base.height - 1);
    uint16_t j;
    for (i = 0; i < self->base.width; i++) {
        for (j = 0; j < self->base.height; j++) {
            uint8_t data[] = {color >> 8, color & 0xFF};
            write_data(self, data, 2);
        }
    }
}
void st7735_fill_16(ST7735ObjectTypeDef *self, lv_color_t color)
{
    uint16_t i;
    st7735_set_window(self, 0, 0, self->base.width - 1, self->base.height - 1);
    uint16_t j;
    //printf("\r\nCOLOR: %0.4X\r\n", color.full);
    //printf("CH_R: %0.2X\tCH_G:%0.2X\tCH_B:%0.2X\r\n", color.ch.red, color.ch.green, color.ch.blue);
    for (i = 0; i < self->base.width; i++) {
        for (j = 0; j < self->base.height; j++) {
            write_data(self, (uint8_t*)&color.full, 2);
        }
    }
}


static void export_st7735_reset(DisplayObjectTypeDef *self_base)
{
	ST7735ObjectTypeDef *self = (ST7735ObjectTypeDef*)self_base;
    self->set_backlight(self, 100);
    st7735_hard_reset(self);
    //st7735_soft_reset(self);
    write_command(self, 0x11); //Sleep out
    self->sleep_ms(self, 50); //Delay 120ms

    //------------------------------------ST7735S Frame rate-------------------------------------------------//
    write_command(self, ST7735_RGBCTRL); //Frame rate 80Hz
    write_a_byte(self, 0x02);
    write_a_byte(self, 0x35);
    write_a_byte(self, 0x36);
    write_command(self, ST7735_PORCTRL); //Frame rate 80Hz
    write_a_byte(self, 0x02);
    write_a_byte(self, 0x35);
    write_a_byte(self, 0x36);
    write_command(self, ST7735_FRCTRL1); //Frame rate 80Hz
    write_a_byte(self, 0x02);
    write_a_byte(self, 0x35);
    write_a_byte(self, 0x36);
    write_a_byte(self, 0x02);
    write_a_byte(self, 0x35);
    write_a_byte(self, 0x36);
    //------------------------------------End ST7735S Frame rate-------------------------------------------//

    // write_command(self, 0xB4); //Dot inversion
    // write_a_byte(self, 0x03);
    /*------------------------------------ST7735S Power Sequence-----------------------------------------*/
    write_command(self, ST7735_LCMCTRL);
    write_a_byte(self, 0xA2);
    write_a_byte(self, 0x02);
    write_a_byte(self, 0x84);
    write_command(self, ST7735_IDSET);
    write_a_byte(self, 0xC5);
    write_command(self, ST7735_VDVVRHEN);
    write_a_byte(self, 0x0D);
    write_a_byte(self, 0x00);
    write_command(self, ST7735_VRHS);
    write_a_byte(self, 0x8D);
    write_a_byte(self, 0x2A);
    write_command(self, ST7735_VDVSET);
    write_a_byte(self, 0x8D);
    write_a_byte(self, 0xEE);

    /* ------------------------------------ST7735S Gamma Setting -----------------------------------------*/
    write_command(self, ST7735_PVGAMCTRL);
    write_a_byte(self, 0xd0);
    write_a_byte(self, 0x15);
    write_a_byte(self, 0x1A);
    write_a_byte(self, 0x0B);
    write_a_byte(self, 0x17);
    write_a_byte(self, 0x2A);
    write_a_byte(self, 0x3F);
    write_a_byte(self, 0x55);
    write_a_byte(self, 0x4C);
    write_a_byte(self, 0x2F);
    write_a_byte(self, 0x1F);
    write_a_byte(self, 0x1F);
    write_a_byte(self, 0x20);
    write_a_byte(self, 0x23);

    write_command(self, ST7735_NVGAMCTRL);
    write_a_byte(self, 0xd0);
    write_a_byte(self, 0x15);
    write_a_byte(self, 0x10);
    write_a_byte(self, 0x0B);
    write_a_byte(self, 0x17);
    write_a_byte(self, 0x2A);
    write_a_byte(self, 0x3F);
    write_a_byte(self, 0x44);
    write_a_byte(self, 0x51);
    write_a_byte(self, 0x2F);
    write_a_byte(self, 0x1F);
    write_a_byte(self, 0x1F);
    write_a_byte(self, 0x20);
    write_a_byte(self, 0x23);

    st7735_set_color_mode(self, COLOR_MODE_16BIT);
    st7735_rotation(self, 3);
    st7735_inversion_mode(self, true);

    write_command(self, ST7735_NORON);
    st7735_fill_16(self, LV_COLOR_ORANGE);
    write_command(self, ST7735_DISPON);

    ((ST7735ObjectTypeDef *)self)->set_backlight(self, 100);
}

static void export_st7735_display_on(DisplayObjectTypeDef *self_base)
{
    write_command((ST7735ObjectTypeDef*)self_base, ST7735_DISPON);
}

static void export_st7735_display_off(DisplayObjectTypeDef *self_base)
{
    write_command((ST7735ObjectTypeDef*)self_base, ST7735_DISPOFF);
}

static void export_st7735_set_backlight(DisplayObjectTypeDef *self, uint32_t brightness)
{
    ((ST7735ObjectTypeDef*)self)->set_backlight(((ST7735ObjectTypeDef*)self), brightness);
}

void st7735_object_init(ST7735ObjectTypeDef *self)
{
    self->base.draw_pixel = export_st7735_draw_pixel;
    self->base.draw_bitmap = export_st7735_draw_bitmap;
    self->base.reset = export_st7735_reset;
    self->base.display_on = export_st7735_display_on;
    self->base.display_off =  export_st7735_display_off;
    self->base.set_backlight =  export_st7735_set_backlight;
}
