#include "spi.h"
#include "cmsis_os2.h"

#include "lwmem_porting.h"
#include "display_st7735.h"

/* 全局变量 */
DisplayObjectTypeDef *lcd;

// LCD 的适配接口
static void lcd1_spi_write(ST7735ObjectTypeDef *self, uint8_t *data, size_t len);
static void lcd1_set_dc_cs(ST7735ObjectTypeDef *self, uint32_t new_dc, uint32_t new_cs);
static void lcd1_set_res(ST7735ObjectTypeDef *self, uint32_t new_res);
static void lcd1_set_blacklight(ST7735ObjectTypeDef *self, uint32_t brightness);
static void DelayMs(ST7735ObjectTypeDef *self, uint32_t ms);

void lcds_init(void)
{
    ST7735ObjectTypeDef *st7735 = LWMEM_CCM_MALLOC(sizeof(ST7735ObjectTypeDef));
    st7735_object_init(st7735);
    st7735->base.height = 80;
    st7735->base.width = 160;
    st7735->x_offset = 1;
    st7735->y_offset = 26;
    st7735->set_backlight = lcd1_set_blacklight;
    st7735->set_dc_cs = lcd1_set_dc_cs;
    st7735->set_res = lcd1_set_res;
    st7735->spi_write = lcd1_spi_write,
    st7735->sleep_ms = DelayMs;
    st7735->set_dc_cs(st7735, 1, 1);
    lcd = (DisplayObjectTypeDef*)st7735;
}


static void DelayMs(ST7735ObjectTypeDef *self, uint32_t ms)
{
    osDelay(ms);
}

void tx_complete_callback(SPI_HandleTypeDef *arg) {
		extern osSemaphoreId_t spi_tx_finishedHandle;
		HAL_SPI_DMAStop(&hspi2);
		osSemaphoreRelease(spi_tx_finishedHandle);
}

// LCD 的适配接口
static void lcd1_spi_write(ST7735ObjectTypeDef *self, uint8_t *data, size_t len)
{
	extern osSemaphoreId_t spi_tx_finishedHandle;
	HAL_SPI_Transmit(&hspi2, data, len, 0xFFFF);
	//HAL_SPI_Transmit(&hspi2, 
	//osSemaphoreAcquire(spi_tx_finishedHandle, osWaitForever);
	//HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_TX_COMPLETE_CB_ID, tx_complete_callback);
	//HAL_SPI_Transmit_DMA(&hspi2, data, len);
	//HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, buff, buff_size);
	//while(hspi2.State == HAL_SPI_STATE_BUSY_TX);
}

static void lcd1_set_dc_cs(ST7735ObjectTypeDef *self, uint32_t new_dc, uint32_t new_cs)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, (GPIO_PinState)new_cs);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, (GPIO_PinState)new_dc);
}

static void lcd1_set_res(ST7735ObjectTypeDef *self, uint32_t new_res)
{
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, (GPIO_PinState)new_res);
}

static void lcd1_set_blacklight(ST7735ObjectTypeDef *self, uint32_t brightness)
{
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, brightness > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

