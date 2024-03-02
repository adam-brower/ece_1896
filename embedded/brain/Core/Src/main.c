/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @brief   BLE application with BLE core
 *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                    ##### IMPORTANT NOTE #####
  ==============================================================================

  This application requests having the stm32wb5x_BLE_Stack_fw.bin binary
  flashed on the Wireless Coprocessor.
  If it is not the case, you need to use STM32CubeProgrammer to load the appropriate
  binary.

  All available binaries are located under following directory:
  /Projects/STM32_Copro_Wireless_Binaries

  Refer to UM2237 to learn how to use/install STM32CubeProgrammer.
  Refer to /Projects/STM32_Copro_Wireless_Binaries/ReleaseNote.html for the
  detailed procedure to change the Wireless Coprocessor binary.

  @endverbatim
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "Adafruit_RA8875.h"

#include <string.h>
#include <stdarg.h> //for va_list var arg functions

#include "app_fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IPCC_HandleTypeDef hipcc;
RNG_HandleTypeDef hrng;
RTC_HandleTypeDef hrtc;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

SAI_HandleTypeDef hsai_BlockA1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_SAI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */

static void MX_IPCC_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

uint32_t count;


// ----------------------------------------------------
// NOTE:
// lvgl driver port to stm32wb

#include "stm32wbxx_hal_spi.h"
//#include ""
#include "lvgl/lvgl.h"

//...

lv_display_t *my_disp;
volatile int my_disp_bus_busy = 0;

//...

/* DMA transfer ready callback */
static void my_lcd_color_transfer_ready_cb(SPI_HandleTypeDef *hspi)
{
        /* CS high */
        HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
        my_disp_bus_busy = 0;
        lv_display_flush_ready(my_disp);
}

/* Initialize LCD I/O bus, reset LCD */
static int32_t my_lcd_io_init(void)
{
	/* Register SPI Tx Complete Callback */
	 HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_COMPLETE_CB_ID, my_lcd_color_transfer_ready_cb);

	/* reset LCD */
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);

	// TODO: Determine if DCX is needed. I don't think so.
	// HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);

	return HAL_OK;
}

/* Send short command to the LCD. This function shall wait until the transaction finishes. */
static void my_lcd_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size)
{
        LV_UNUSED(disp);
        /* Set the SPI in 8-bit mode */
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        HAL_SPI_Init(&hspi1);
        /* DCX low (command) */
        // HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_RESET);
        /* CS low */
        HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
        /* send command */
        if (HAL_SPI_Transmit(&hspi1, cmd, cmd_size, HAL_MAX_DELAY) == HAL_OK) {
                /* DCX high (data) */
                // HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
                /* for short data blocks we use polling transfer */
                HAL_SPI_Transmit(&hspi1, (uint8_t *)param, (uint16_t)param_size, HAL_MAX_DELAY);
                /* CS high */
                HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
        }
}

///* Send large array of pixel data to the LCD. If necessary, this function has to do the byte-swapping. This function can do the transfer in the background. */
static void my_lcd_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size)
{
        LV_UNUSED(disp);
        while (my_disp_bus_busy);       /* wait until previous transfer is finished */
        /* Set the SPI in 8-bit mode */
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        HAL_SPI_Init(&hspi1);
        /* DCX low (command) */

        // TODO: determine if needed
        // HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_RESET);
        /* CS low */
        HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
        /* send command */
        if (HAL_SPI_Transmit(&hspi1, cmd, cmd_size, HAL_MAX_DELAY) == HAL_OK) {
                /* DCX high (data) */
        		// TODO: determine if needed// TODO: determine if needed
                // HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
                /* for color data use DMA transfer */
                /* Set the SPI in 16-bit mode to match endianess */
                hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
                HAL_SPI_Init(&hspi1);
                my_disp_bus_busy = 1;
                HAL_SPI_Transmit_DMA(&hspi1, param, (uint16_t)param_size / 2);

                /* NOTE: CS will be reset in the transfer ready callback */
        }
}

//Frame buffers
/*Static or global buffer(s). The second buffer is optional*/
void Adafruit_RA8875_drawPixel(int16_t x, int16_t y, uint16_t color);

void my_flush_cb(lv_display_t * disp, const lv_area_t * area, void * px_map)
{
  //Set the drawing region
	// TODO: determine if needed
//  set_draw_window(area->x1, area->y1, area->x2, area->y2);

  int height = area->y2 - area->y1 + 1;
  int width = area->x2 - area->x1 + 1;

  uint16_t * buf16 = (uint16_t *)px_map;

  //We will do the SPI write manually here for speed
//  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
  //CS low to begin data
  HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);

  //Write colour to each pixel
  int current_height = area->y1;
  for (int i = 0; i < height; i++) {
	  Adafruit_RA8875_drawPixels(buf16, width ,area->x1, current_height);
	  buf16 += width;
	  current_height++;
  }

  //Return CS to high
  HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);

  /* IMPORTANT!!!
  * Inform the graphics library that you are ready with the flushing*/
  lv_display_flush_ready(disp);
}


// ----------------------------------------------------
// Adafruit-Ra8875 Library ported to C
// TODO: move this to another file and make it not ugly code

// ----------------------------------------------------
// Adafruit-Ra8875 - Implementation (.h)

/**************************************************************************/
/*!
    @file     Adafruit_RA8875.h
    @author   Limor Friend/Ladyada, K.Townsend/KTOWN for Adafruit Industries

     This is the library for the Adafruit RA8875 Driver board for TFT displays
     ---------------> http://www.adafruit.com/products/1590
     The RA8875 is a TFT driver for up to 800x480 dotclock'd displays
     It is tested to work with displays in the Adafruit shop. Other displays
     may need timing adjustments and are not guanteed to work.

     Adafruit invests time and resources providing this open
     source code, please support Adafruit and open-source hardware
     by purchasing products from Adafruit!

     Written by Limor Fried/Ladyada for Adafruit Industries.
     BSD license, check license.txt for more information.
     All text above must be included in any redistribution.
*/
/**************************************************************************/






// Touchscreen Calibration and EEPROM Storage Defines
#define CFG_EEPROM_TOUCHSCREEN_CAL_AN 0       ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_BN 4       ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_CN 8       ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_DN 12      ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_EN 16      ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_FN 20      ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_DIVIDER 24 ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CALIBRATED 28  ///< EEPROM Storage Location


/**************************************************************************/
/*!
 @enum RA8875sizes The Supported Screen Sizes
 */
/**************************************************************************/
enum RA8875sizes {
  RA8875_480x80,  /*!<  480x80 Pixel Display */
  RA8875_480x128, /*!< 480x128 Pixel Display */
  RA8875_480x272, /*!< 480x272 Pixel Display */
  RA8875_800x480  /*!< 800x480 Pixel Display */
};

/**************************************************************************/
/*!
 @struct Point
 Calibration Point

 @var Point::x
    x-coordinate
 @var Point::y
    y-coordinate
 */
/**************************************************************************/
typedef struct Point {
  int32_t x;
  int32_t y;
} tsPoint_t; ///< Nameless struct variable!

/**************************************************************************/
/*!
 @struct tsMatrix_t
 Calibration Data Structure

 @var tsMatrix_t::An
 A Coefficient with the coarsest granularity
 @var tsMatrix_t::Bn
 B Coeffiecient
 @var tsMatrix_t::Cn
 C Coefficient
 @var tsMatrix_t::Dn
 D Coeffiecient
 @var tsMatrix_t::En
 E Coefficient
 @var tsMatrix_t::Fn
 F Coeffiecient with the finest granularity
 @var tsMatrix_t::Divider
 Divider for Coefficients
 */
/**************************************************************************/
typedef struct // Matrix
{
  int32_t An, Bn, Cn, Dn, En, Fn, Divider;
} tsMatrix_t;


// Colors (RGB565)
#define RA8875_BLACK 0x0000   ///< Black Color
#define RA8875_BLUE 0x001F    ///< Blue Color
#define RA8875_RED 0xF800     ///< Red Color
#define RA8875_GREEN 0x07E0   ///< Green Color
#define RA8875_CYAN 0x07FF    ///< Cyan Color
#define RA8875_MAGENTA 0xF81F ///< Magenta Color
#define RA8875_YELLOW 0xFFE0  ///< Yellow Color
#define RA8875_WHITE 0xFFFF   ///< White Color

// Command/Data pins for SPI
#define RA8875_DATAWRITE 0x00 ///< See datasheet
#define RA8875_DATAREAD 0x40  ///< See datasheet
#define RA8875_CMDWRITE 0x80  ///< See datasheet
#define RA8875_CMDREAD 0xC0   ///< See datasheet

// Registers & bits
#define RA8875_PWRR 0x01           ///< See datasheet
#define RA8875_PWRR_DISPON 0x80    ///< See datasheet
#define RA8875_PWRR_DISPOFF 0x00   ///< See datasheet
#define RA8875_PWRR_SLEEP 0x02     ///< See datasheet
#define RA8875_PWRR_NORMAL 0x00    ///< See datasheet
#define RA8875_PWRR_SOFTRESET 0x01 ///< See datasheet

#define RA8875_MRWC 0x02 ///< See datasheet

#define RA8875_GPIOX 0xC7 ///< See datasheet

#define RA8875_PLLC1 0x88         ///< See datasheet
#define RA8875_PLLC1_PLLDIV2 0x80 ///< See datasheet
#define RA8875_PLLC1_PLLDIV1 0x00 ///< See datasheet

#define RA8875_PLLC2 0x89        ///< See datasheet
#define RA8875_PLLC2_DIV1 0x00   ///< See datasheet
#define RA8875_PLLC2_DIV2 0x01   ///< See datasheet
#define RA8875_PLLC2_DIV4 0x02   ///< See datasheet
#define RA8875_PLLC2_DIV8 0x03   ///< See datasheet
#define RA8875_PLLC2_DIV16 0x04  ///< See datasheet
#define RA8875_PLLC2_DIV32 0x05  ///< See datasheet
#define RA8875_PLLC2_DIV64 0x06  ///< See datasheet
#define RA8875_PLLC2_DIV128 0x07 ///< See datasheet

#define RA8875_SYSR 0x10       ///< See datasheet
#define RA8875_SYSR_8BPP 0x00  ///< See datasheet
#define RA8875_SYSR_16BPP 0x0C ///< See datasheet
#define RA8875_SYSR_MCU8 0x00  ///< See datasheet
#define RA8875_SYSR_MCU16 0x03 ///< See datasheet

#define RA8875_PCSR 0x04       ///< See datasheet
#define RA8875_PCSR_PDATR 0x00 ///< See datasheet
#define RA8875_PCSR_PDATL 0x80 ///< See datasheet
#define RA8875_PCSR_CLK 0x00   ///< See datasheet
#define RA8875_PCSR_2CLK 0x01  ///< See datasheet
#define RA8875_PCSR_4CLK 0x02  ///< See datasheet
#define RA8875_PCSR_8CLK 0x03  ///< See datasheet

#define RA8875_HDWR 0x14 ///< See datasheet

#define RA8875_HNDFTR 0x15         ///< See datasheet
#define RA8875_HNDFTR_DE_HIGH 0x00 ///< See datasheet
#define RA8875_HNDFTR_DE_LOW 0x80  ///< See datasheet

#define RA8875_HNDR 0x16      ///< See datasheet
#define RA8875_HSTR 0x17      ///< See datasheet
#define RA8875_HPWR 0x18      ///< See datasheet
#define RA8875_HPWR_LOW 0x00  ///< See datasheet
#define RA8875_HPWR_HIGH 0x80 ///< See datasheet

#define RA8875_VDHR0 0x19     ///< See datasheet
#define RA8875_VDHR1 0x1A     ///< See datasheet
#define RA8875_VNDR0 0x1B     ///< See datasheet
#define RA8875_VNDR1 0x1C     ///< See datasheet
#define RA8875_VSTR0 0x1D     ///< See datasheet
#define RA8875_VSTR1 0x1E     ///< See datasheet
#define RA8875_VPWR 0x1F      ///< See datasheet
#define RA8875_VPWR_LOW 0x00  ///< See datasheet
#define RA8875_VPWR_HIGH 0x80 ///< See datasheet

#define RA8875_HSAW0 0x30 ///< See datasheet
#define RA8875_HSAW1 0x31 ///< See datasheet
#define RA8875_VSAW0 0x32 ///< See datasheet
#define RA8875_VSAW1 0x33 ///< See datasheet

#define RA8875_HEAW0 0x34 ///< See datasheet
#define RA8875_HEAW1 0x35 ///< See datasheet
#define RA8875_VEAW0 0x36 ///< See datasheet
#define RA8875_VEAW1 0x37 ///< See datasheet

#define RA8875_MCLR 0x8E            ///< See datasheet
#define RA8875_MCLR_START 0x80      ///< See datasheet
#define RA8875_MCLR_STOP 0x00       ///< See datasheet
#define RA8875_MCLR_READSTATUS 0x80 ///< See datasheet
#define RA8875_MCLR_FULL 0x00       ///< See datasheet
#define RA8875_MCLR_ACTIVE 0x40     ///< See datasheet

#define RA8875_DCR 0x90                   ///< See datasheet
#define RA8875_DCR_LINESQUTRI_START 0x80  ///< See datasheet
#define RA8875_DCR_LINESQUTRI_STOP 0x00   ///< See datasheet
#define RA8875_DCR_LINESQUTRI_STATUS 0x80 ///< See datasheet
#define RA8875_DCR_CIRCLE_START 0x40      ///< See datasheet
#define RA8875_DCR_CIRCLE_STATUS 0x40     ///< See datasheet
#define RA8875_DCR_CIRCLE_STOP 0x00       ///< See datasheet
#define RA8875_DCR_FILL 0x20              ///< See datasheet
#define RA8875_DCR_NOFILL 0x00            ///< See datasheet
#define RA8875_DCR_DRAWLINE 0x00          ///< See datasheet
#define RA8875_DCR_DRAWTRIANGLE 0x01      ///< See datasheet
#define RA8875_DCR_DRAWSQUARE 0x10        ///< See datasheet

#define RA8875_ELLIPSE 0xA0        ///< See datasheet
#define RA8875_ELLIPSE_STATUS 0x80 ///< See datasheet

#define RA8875_MWCR0 0x40         ///< See datasheet
#define RA8875_MWCR0_GFXMODE 0x00 ///< See datasheet
#define RA8875_MWCR0_TXTMODE 0x80 ///< See datasheet
#define RA8875_MWCR0_CURSOR 0x40  ///< See datasheet
#define RA8875_MWCR0_BLINK 0x20   ///< See datasheet

#define RA8875_MWCR0_DIRMASK 0x0C ///< Bitmask for Write Direction
#define RA8875_MWCR0_LRTD 0x00    ///< Left->Right then Top->Down
#define RA8875_MWCR0_RLTD 0x04    ///< Right->Left then Top->Down
#define RA8875_MWCR0_TDLR 0x08    ///< Top->Down then Left->Right
#define RA8875_MWCR0_DTLR 0x0C    ///< Down->Top then Left->Right

#define RA8875_BTCR 0x44  ///< See datasheet
#define RA8875_CURH0 0x46 ///< See datasheet
#define RA8875_CURH1 0x47 ///< See datasheet
#define RA8875_CURV0 0x48 ///< See datasheet
#define RA8875_CURV1 0x49 ///< See datasheet

#define RA8875_P1CR 0x8A         ///< See datasheet
#define RA8875_P1CR_ENABLE 0x80  ///< See datasheet
#define RA8875_P1CR_DISABLE 0x00 ///< See datasheet
#define RA8875_P1CR_CLKOUT 0x10  ///< See datasheet
#define RA8875_P1CR_PWMOUT 0x00  ///< See datasheet

#define RA8875_P1DCR 0x8B ///< See datasheet

#define RA8875_P2CR 0x8C         ///< See datasheet
#define RA8875_P2CR_ENABLE 0x80  ///< See datasheet
#define RA8875_P2CR_DISABLE 0x00 ///< See datasheet
#define RA8875_P2CR_CLKOUT 0x10  ///< See datasheet
#define RA8875_P2CR_PWMOUT 0x00  ///< See datasheet

#define RA8875_P2DCR 0x8D ///< See datasheet

#define RA8875_PWM_CLK_DIV1 0x00     ///< See datasheet
#define RA8875_PWM_CLK_DIV2 0x01     ///< See datasheet
#define RA8875_PWM_CLK_DIV4 0x02     ///< See datasheet
#define RA8875_PWM_CLK_DIV8 0x03     ///< See datasheet
#define RA8875_PWM_CLK_DIV16 0x04    ///< See datasheet
#define RA8875_PWM_CLK_DIV32 0x05    ///< See datasheet
#define RA8875_PWM_CLK_DIV64 0x06    ///< See datasheet
#define RA8875_PWM_CLK_DIV128 0x07   ///< See datasheet
#define RA8875_PWM_CLK_DIV256 0x08   ///< See datasheet
#define RA8875_PWM_CLK_DIV512 0x09   ///< See datasheet
#define RA8875_PWM_CLK_DIV1024 0x0A  ///< See datasheet
#define RA8875_PWM_CLK_DIV2048 0x0B  ///< See datasheet
#define RA8875_PWM_CLK_DIV4096 0x0C  ///< See datasheet
#define RA8875_PWM_CLK_DIV8192 0x0D  ///< See datasheet
#define RA8875_PWM_CLK_DIV16384 0x0E ///< See datasheet
#define RA8875_PWM_CLK_DIV32768 0x0F ///< See datasheet

#define RA8875_TPCR0 0x70               ///< See datasheet
#define RA8875_TPCR0_ENABLE 0x80        ///< See datasheet
#define RA8875_TPCR0_DISABLE 0x00       ///< See datasheet
#define RA8875_TPCR0_WAIT_512CLK 0x00   ///< See datasheet
#define RA8875_TPCR0_WAIT_1024CLK 0x10  ///< See datasheet
#define RA8875_TPCR0_WAIT_2048CLK 0x20  ///< See datasheet
#define RA8875_TPCR0_WAIT_4096CLK 0x30  ///< See datasheet
#define RA8875_TPCR0_WAIT_8192CLK 0x40  ///< See datasheet
#define RA8875_TPCR0_WAIT_16384CLK 0x50 ///< See datasheet
#define RA8875_TPCR0_WAIT_32768CLK 0x60 ///< See datasheet
#define RA8875_TPCR0_WAIT_65536CLK 0x70 ///< See datasheet
#define RA8875_TPCR0_WAKEENABLE 0x08    ///< See datasheet
#define RA8875_TPCR0_WAKEDISABLE 0x00   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV1 0x00   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV2 0x01   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV4 0x02   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV8 0x03   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV16 0x04  ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV32 0x05  ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV64 0x06  ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV128 0x07 ///< See datasheet

#define RA8875_TPCR1 0x71            ///< See datasheet
#define RA8875_TPCR1_AUTO 0x00       ///< See datasheet
#define RA8875_TPCR1_MANUAL 0x40     ///< See datasheet
#define RA8875_TPCR1_VREFINT 0x00    ///< See datasheet
#define RA8875_TPCR1_VREFEXT 0x20    ///< See datasheet
#define RA8875_TPCR1_DEBOUNCE 0x04   ///< See datasheet
#define RA8875_TPCR1_NODEBOUNCE 0x00 ///< See datasheet
#define RA8875_TPCR1_IDLE 0x00       ///< See datasheet
#define RA8875_TPCR1_WAIT 0x01       ///< See datasheet
#define RA8875_TPCR1_LATCHX 0x02     ///< See datasheet
#define RA8875_TPCR1_LATCHY 0x03     ///< See datasheet

#define RA8875_TPXH 0x72  ///< See datasheet
#define RA8875_TPYH 0x73  ///< See datasheet
#define RA8875_TPXYL 0x74 ///< See datasheet

#define RA8875_INTC1 0xF0     ///< See datasheet
#define RA8875_INTC1_KEY 0x10 ///< See datasheet
#define RA8875_INTC1_DMA 0x08 ///< See datasheet
#define RA8875_INTC1_TP 0x04  ///< See datasheet
#define RA8875_INTC1_BTE 0x02 ///< See datasheet

#define RA8875_INTC2 0xF1     ///< See datasheet
#define RA8875_INTC2_KEY 0x10 ///< See datasheet
#define RA8875_INTC2_DMA 0x08 ///< See datasheet
#define RA8875_INTC2_TP 0x04  ///< See datasheet
#define RA8875_INTC2_BTE 0x02 ///< See datasheet

#define RA8875_SCROLL_BOTH 0x00   ///< See datasheet
#define RA8875_SCROLL_LAYER1 0x40 ///< See datasheet
#define RA8875_SCROLL_LAYER2 0x80 ///< See datasheet
#define RA8875_SCROLL_BUFFER 0xC0 ///< See datasheet

  bool Adafruit_RA8875_begin();
  void Adafruit_RA8875_softReset(void);
  void Adafruit_RA8875_displayOn(bool on);
  void Adafruit_RA8875_sleep(bool sleep);

  /* Text functions */
  void Adafruit_RA8875_textMode(void);
  void Adafruit_RA8875_textSetCursor(uint16_t x, uint16_t y);
  void Adafruit_RA8875_textColor(uint16_t foreColor, uint16_t bgColor);
  void Adafruit_RA8875_textTransparent(uint16_t foreColor);
  void Adafruit_RA8875_textEnlarge(uint8_t scale);
  void Adafruit_RA8875_textWrite(const char *buffer, uint16_t len );
  void Adafruit_RA8875_cursorBlink(uint8_t rate);

  /* Graphics functions */
  void Adafruit_RA8875_graphicsMode(void);
  void Adafruit_RA8875_setXY(uint16_t x, uint16_t y);
  void Adafruit_RA8875_pushPixels(uint32_t num, uint16_t p);

  /* Adafruit_GFX functions */
//  NOTE: Declared above
//  void Adafruit_RA8875_drawPixel(int16_t x, int16_t y, uint16_t color);
  void Adafruit_RA8875_drawPixels(uint16_t *p, uint32_t count, int16_t x, int16_t y);
  void Adafruit_RA8875_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void Adafruit_RA8875_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

  /* HW accelerated wrapper functions (override Adafruit_GFX prototypes) */
  void Adafruit_RA8875_fillScreen(uint16_t color);
  void Adafruit_RA8875_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void Adafruit_RA8875_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void Adafruit_RA8875_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void Adafruit_RA8875_drawCircle(int16_t x, int16_t y, int16_t r, uint16_t color);
  void Adafruit_RA8875_fillCircle(int16_t x, int16_t y, int16_t r, uint16_t color);
  void Adafruit_RA8875_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, uint16_t color);
  void Adafruit_RA8875_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, uint16_t color);
  void Adafruit_RA8875_drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis,
                   int16_t shortAxis, uint16_t color);
  void Adafruit_RA8875_fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis,
                   int16_t shortAxis, uint16_t color);
  void Adafruit_RA8875_drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis,
                 int16_t shortAxis, uint8_t curvePart, uint16_t color);
  void Adafruit_RA8875_fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis,
                 int16_t shortAxis, uint8_t curvePart, uint16_t color);
  void Adafruit_RA8875_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,
                     uint16_t color);
  void Adafruit_RA8875_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,
                     uint16_t color);

  /* Scroll */
  void Adafruit_RA8875_setScrollWindow(int16_t x, int16_t y, int16_t w, int16_t h,
                       uint8_t mode);
  void Adafruit_RA8875_scrollX(int16_t dist);
  void Adafruit_RA8875_scrollY(int16_t dist);

  /* Backlight */
  void Adafruit_RA8875_GPIOX(bool on);
  void Adafruit_RA8875_PWM1config(bool on, uint8_t clock);
  void Adafruit_RA8875_PWM2config(bool on, uint8_t clock);
  void Adafruit_RA8875_PWM1out(uint8_t p);
  void Adafruit_RA8875_PWM2out(uint8_t p);

  /* Touch screen */
  void Adafruit_RA8875_touchEnable(bool on);
  bool Adafruit_RA8875_touched(void);
  bool Adafruit_RA8875_touchRead(uint16_t *x, uint16_t *y);


  /* Low level access */
  void Adafruit_RA8875_writeReg(uint8_t reg, uint8_t val);
  uint8_t Adafruit_RA8875_readReg(uint8_t reg);
  void Adafruit_RA8875_writeData(uint8_t d);
  uint8_t Adafruit_RA8875_readData(void);
  void Adafruit_RA8875_writeCommand(uint8_t d);
  uint8_t Adafruit_RA8875_readStatus(void);
  bool Adafruit_RA8875_waitPoll(uint8_t r, uint8_t f);
  uint16_t Adafruit_RA8875_width(void);
  uint16_t Adafruit_RA8875_height(void);
  void Adafruit_RA8875_setRotation(int8_t rotation);
  int8_t Adafruit_RA8875_getRotation(void);

  // "Private" Functions from library
  void Adafruit_RA8875_PLLinit(void);
  void Adafruit_RA8875_initialize(void);

  /* GFX Helper Functions */
  void Adafruit_RA8875_circleHelper(int16_t x, int16_t y, int16_t r, uint16_t color,
                    bool filled);
  void Adafruit_RA8875_rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color,
                  bool filled);
  void Adafruit_RA8875_triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                      int16_t x2, int16_t y2, uint16_t color, bool filled);
  void Adafruit_RA8875_ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis,
                     int16_t shortAxis, uint16_t color, bool filled);
  void Adafruit_RA8875_curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis,
                   int16_t shortAxis, uint8_t curvePart, uint16_t color,
                   bool filled);
  void Adafruit_RA8875_roundRectHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,
                       uint16_t color, bool filled);

  /* Rotation Functions */
  int16_t Adafruit_RA8875_applyRotationX(int16_t x);
  int16_t Adafruit_RA8875_applyRotationY(int16_t y);


// ----------------------------------------------------
// Adafruit-Ra8875 - Implementation (.c)

uint8_t _rst;
SPI_HandleTypeDef _DIS_HSPI;
enum RA8875sizes _size;
uint16_t _width, _height;
uint8_t _textScale;
uint8_t _rotation;
uint8_t _voffset;


/**************************************************************************/
/*!
            Constructor for a new RA8875 instance

            @param CS    Location of the SPI chip select pin
            @param RST Location of the reset pin
*/
/**************************************************************************/
void Adafruit_RA8875(uint8_t RST, SPI_HandleTypeDef DIS_HSPI) {
  _DIS_HSPI = DIS_HSPI;
  _rst = RST;
}

/**************************************************************************/
/*!
            Initialises the LCD driver and any HW required by the display

            @param s The display size, which can be either:
                                    'RA8875_480x80'    (3.8" displays) or
                                    'RA8875_480x128' (3.9" displays) or
                                    'RA8875_480x272' (4.3" displays) or
                                    'RA8875_800x480' (5" and 7" displays)

            @return True if we reached the end
//*/
///**************************************************************************/
bool Adafruit_RA8875_begin() {
    _size = RA8875_800x480;
    _width = 800;
    _height = 480;
    _rotation = 0;

    // TODO: init pins manually
      HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);

      // TODO: figure out if needed
//    pinMode(_rst, OUTPUT);
//    digitalWrite(_rst, LOW);
//    delay(100);
//    digitalWrite(_rst, HIGH);
//    delay(100);

//
//    SPI.begin();

    uint8_t x = Adafruit_RA8875_readReg(0);
    printf("x = %d\r\n", x);
    if (x != 0x75) {
		printf("DISPLAY READ REGISTER FAILED!!!! \r\n");
        return false;
    }

    Adafruit_RA8875_initialize();
    return true;
}

/************************* Initialization *********************************/

/**************************************************************************/
/*!
            Performs a SW-based reset of the RA8875
*/
/**************************************************************************/
void Adafruit_RA8875_softReset(void) {
    Adafruit_RA8875_writeCommand(RA8875_PWRR);
    Adafruit_RA8875_writeData(RA8875_PWRR_SOFTRESET);
    Adafruit_RA8875_writeData(RA8875_PWRR_NORMAL);
    HAL_Delay(1);
}

/**************************************************************************/
/*!
            Initialise the PLL
*/
/**************************************************************************/
void Adafruit_RA8875_PLLinit(void) {
    // _size == RA8875_800x480
    Adafruit_RA8875_writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
    HAL_Delay(1);
    Adafruit_RA8875_writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    HAL_Delay(1);
}

/**************************************************************************/
/*!
            Initialises the driver IC (clock setup, etc.)
*/
/**************************************************************************/
void Adafruit_RA8875_initialize(void) {
    Adafruit_RA8875_PLLinit();
    Adafruit_RA8875_writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

    /* Timing values */
    uint8_t pixclk;
    uint8_t hsync_start;
    uint8_t hsync_pw;
    uint8_t hsync_finetune;
    uint8_t hsync_nondisp;
    uint8_t vsync_pw;
    uint16_t vsync_nondisp;
    uint16_t vsync_start;

    // (_size == RA8875_800x480)
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp = 26;
    hsync_start = 32;
    hsync_pw = 96;
    hsync_finetune = 0;
    vsync_nondisp = 32;
    vsync_start = 23;
    vsync_pw = 2;
    _voffset = 0;

    Adafruit_RA8875_writeReg(RA8875_PCSR, pixclk);
    HAL_Delay(1);

    /* Horizontal settings registers */
    Adafruit_RA8875_writeReg(RA8875_HDWR, (_width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
    Adafruit_RA8875_writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
    Adafruit_RA8875_writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                                                        8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
    Adafruit_RA8875_writeReg(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
    Adafruit_RA8875_writeReg(RA8875_HPWR,
                     RA8875_HPWR_LOW +
                             (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

    /* Vertical settings registers */
    Adafruit_RA8875_writeReg(RA8875_VDHR0, (uint16_t)(_height - 1 + _voffset) & 0xFF);
    Adafruit_RA8875_writeReg(RA8875_VDHR1, (uint16_t)(_height - 1 + _voffset) >> 8);
    Adafruit_RA8875_writeReg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
    Adafruit_RA8875_writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
    Adafruit_RA8875_writeReg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
    Adafruit_RA8875_writeReg(RA8875_VSTR1, vsync_start >> 8);
    Adafruit_RA8875_writeReg(RA8875_VPWR,
                     RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

    /* Set active window X */
    Adafruit_RA8875_writeReg(RA8875_HSAW0, 0); // horizontal start point
    Adafruit_RA8875_writeReg(RA8875_HSAW1, 0);
    Adafruit_RA8875_writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF); // horizontal end point
    Adafruit_RA8875_writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);

    /* Set active window Y */
    Adafruit_RA8875_writeReg(RA8875_VSAW0, 0 + _voffset); // vertical start point
    Adafruit_RA8875_writeReg(RA8875_VSAW1, 0 + _voffset);
    Adafruit_RA8875_writeReg(RA8875_VEAW0,
                     (uint16_t)(_height - 1 + _voffset) & 0xFF); // vertical end point
    Adafruit_RA8875_writeReg(RA8875_VEAW1, (uint16_t)(_height - 1 + _voffset) >> 8);

    /* Clear the entire window */
    Adafruit_RA8875_writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
    HAL_Delay(500);
}

/**************************************************************************/
/*!
            Returns the display width in pixels

            @return    The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875_width(void) { return _width; }

/**************************************************************************/
/*!
            Returns the display height in pixels

            @return    The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875_height(void) { return _height; }

/**************************************************************************/
/*!
 Returns the current rotation (0-3)

 @return    The Rotation Setting
 */
/**************************************************************************/
int8_t Adafruit_RA8875_getRotation(void) { return _rotation; }

/**************************************************************************/
/*!
 Sets the current rotation (0-3)

 @param rotation The Rotation Setting
 */
/**************************************************************************/
void Adafruit_RA8875_setRotation(int8_t rotation) {
    switch (rotation) {
    case 2:
        _rotation = rotation;
        break;
    default:
        _rotation = 0;
        break;
    }
}

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
            Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void Adafruit_RA8875_textMode(void) {
    /* Set text mode */
    Adafruit_RA8875_writeCommand(RA8875_MWCR0);
    uint8_t temp = readData();
    temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
    Adafruit_RA8875_writeData(temp);

    /* Select the internal (ROM) font */
    Adafruit_RA8875_writeCommand(0x21);
    temp = Adafruit_RA8875_readData();
    temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
    Adafruit_RA8875_writeData(temp);
}

/**************************************************************************/
/*!
            Sets the display in text mode (as opposed to graphics mode)

            @param x The x position of the cursor (in pixels, 0..1023)
            @param y The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void Adafruit_RA8875_textSetCursor(uint16_t x, uint16_t y) {
    x = Adafruit_RA8875_applyRotationX(x);
    y = Adafruit_RA8875_applyRotationY(y);

    /* Set cursor location */
    Adafruit_RA8875_writeCommand(0x2A);
    Adafruit_RA8875_writeData(x & 0xFF);
    Adafruit_RA8875_writeCommand(0x2B);
    Adafruit_RA8875_writeData(x >> 8);
    Adafruit_RA8875_writeCommand(0x2C);
    Adafruit_RA8875_writeData(y & 0xFF);
    Adafruit_RA8875_writeCommand(0x2D);
    Adafruit_RA8875_writeData(y >> 8);
}

/**************************************************************************/
/*!
            Sets the fore and background color when rendering text

            @param foreColor The RGB565 color to use when rendering the text
            @param bgColor     The RGB565 colot to use for the background
*/
/**************************************************************************/
void Adafruit_RA8875_textColor(uint16_t foreColor, uint16_t bgColor) {
    /* Set Fore Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((foreColor & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((foreColor & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((foreColor & 0x001f));

    /* Set Background Color */
    Adafruit_RA8875_writeCommand(0x60);
    Adafruit_RA8875_writeData((bgColor & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x61);
    Adafruit_RA8875_writeData((bgColor & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x62);
    Adafruit_RA8875_writeData((bgColor & 0x001f));

    /* Clear transparency flag */
    Adafruit_RA8875_writeCommand(0x22);
    uint8_t temp = Adafruit_RA8875_readData();
    temp &= ~(1 << 6); // Clear bit 6
    Adafruit_RA8875_writeData(temp);
}

/**************************************************************************/
/*!
            Sets the fore color when rendering text with a transparent bg

            @param foreColor The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void Adafruit_RA8875_textTransparent(uint16_t foreColor) {
    /* Set Fore Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((foreColor & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((foreColor & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((foreColor & 0x001f));

    /* Set transparency flag */
    Adafruit_RA8875_writeCommand(0x22);
    uint8_t temp = Adafruit_RA8875_readData();
    temp |= (1 << 6); // Set bit 6
    Adafruit_RA8875_writeData(temp);
}

/**************************************************************************/
/*!
            Sets the text enlarge settings, using one of the following values:

            0 = 1x zoom
            1 = 2x zoom
            2 = 3x zoom
            3 = 4x zoom

            @param scale     The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void Adafruit_RA8875_textEnlarge(uint8_t scale) {
    if (scale > 3)
        scale = 3; // highest setting is 3

    /* Set font size flags */
    Adafruit_RA8875_writeCommand(0x22);
    uint8_t temp = Adafruit_RA8875_readData();
    temp &= ~(0xF); // Clears bits 0..3
    temp |= scale << 2;
    temp |= scale;

    Adafruit_RA8875_writeData(temp);

    _textScale = scale;
}

/**************************************************************************/
/*!
         Enable Cursor Visibility and Blink
         Here we set bits 6 and 5 in 40h
         As well as the set the blink rate in 44h
         The rate is 0 through max 255
         the lower the number the faster it blinks (00h is 1 frame time,
         FFh is 256 Frames time.
         Blink Time (sec) = BTCR[44h]x(1/Frame_rate)

         @param rate The frame rate to blink
 */
/**************************************************************************/

void Adafruit_RA8875_cursorBlink(uint8_t rate) {

    Adafruit_RA8875_writeCommand(RA8875_MWCR0);
    uint8_t temp = Adafruit_RA8875_readData();
    temp |= RA8875_MWCR0_CURSOR;
    Adafruit_RA8875_writeData(temp);

    Adafruit_RA8875_writeCommand(RA8875_MWCR0);
    temp = Adafruit_RA8875_readData();
    temp |= RA8875_MWCR0_BLINK;
    Adafruit_RA8875_writeData(temp);

    if (rate > 255)
        rate = 255;
    Adafruit_RA8875_writeCommand(RA8875_BTCR);
    Adafruit_RA8875_writeData(rate);
}

/**************************************************************************/
/*!
            Renders some text on the screen when in text mode

            @param buffer        The buffer containing the characters to render
            @param len             The size of the buffer in bytes
*/
/**************************************************************************/
void Adafruit_RA8875_textWrite(const char *buffer, uint16_t len) {
    if (len == 0)
        len = strlen(buffer);
    Adafruit_RA8875_writeCommand(RA8875_MRWC);
    for (uint16_t i = 0; i < len; i++) {
        Adafruit_RA8875_writeData(buffer[i]);
/// @cond DISABLE
#if defined(__arm__)
        /// @endcond
        // This delay is needed with textEnlarge(1) because
        // Teensy 3.X is much faster than Arduino Uno
        if (_textScale > 0)
            HAL_Delay(1);
/// @cond DISABLE
#else
        /// @endcond
        // For others, delay starting with textEnlarge(2)
        if (_textScale > 1)
            HAL_Delay(1);
/// @cond DISABLE
#endif
        /// @endcond
    }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
            Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void Adafruit_RA8875_graphicsMode(void) {
    Adafruit_RA8875_writeCommand(RA8875_MWCR0);
    uint8_t temp = Adafruit_RA8875_readData();
    temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
    Adafruit_RA8875_writeData(temp);
}

/**************************************************************************/
/*!
            Waits for screen to finish by polling the status!

            @param regname The register name to check
            @param waitflag The value to wait for the status register to match

            @return True if the expected status has been reached
*/
/**************************************************************************/
bool Adafruit_RA8875_waitPoll(uint8_t regname, uint8_t waitflag) {
    /* Wait for the command to finish */
    while (1) {
        uint8_t temp = Adafruit_RA8875_readReg(regname);
        if (!(temp & waitflag))
            return true;
    }
    return false; // MEMEFIX: yeah i know, unreached! - add timeout?
}

/**************************************************************************/
/*!
            Sets the current X/Y position on the display before drawing

            @param x The 0-based x location
            @param y The 0-base y location
*/
/**************************************************************************/
void Adafruit_RA8875_setXY(uint16_t x, uint16_t y) {
    Adafruit_RA8875_writeReg(RA8875_CURH0, x);
    Adafruit_RA8875_writeReg(RA8875_CURH1, x >> 8);
    Adafruit_RA8875_writeReg(RA8875_CURV0, y);
    Adafruit_RA8875_writeReg(RA8875_CURV1, y >> 8);
}

/**************************************************************************/
/*!
            HW accelerated function to push a chunk of raw pixel data

            @param num The number of pixels to push
            @param p     The pixel color to use
*/
/**************************************************************************/
void Adafruit_RA8875_pushPixels(uint32_t num, uint16_t p) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    SPI.transfer(RA8875_DATAWRITE);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);

    while (num--) {
//        SPI.transfer(p >> 8);
    	uint8_t pHigh = p >> 8;
    	HAL_SPI_Transmit(&_DIS_HSPI, &pHigh, 1, HAL_MAX_DELAY);
//    	SPI.transfer(p);
    	uint8_t pLow = p&0xFF;
    	HAL_SPI_Transmit(&_DIS_HSPI, &pLow, 1, HAL_MAX_DELAY);
    }
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
        Fill the screen with the current color
*/
/**************************************************************************/

//NOTE: no function overloading in C
//void Adafruit_RA8875_fillRect(void) {
//    Adafruit_RA8875_writeCommand(RA8875_DCR);
//    Adafruit_RA8875_writeData(RA8875_DCR_LINESQUTRI_STOP | RA8875_DCR_DRAWSQUARE);
//    Adafruit_RA8875_writeData(RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL |
//                        RA8875_DCR_DRAWSQUARE);
//}

/**************************************************************************/
/*!
        Apply current rotation in the X direction

        @return the X value with current rotation applied
 */
/**************************************************************************/
int16_t Adafruit_RA8875_applyRotationX(int16_t x) {
    switch (_rotation) {
    case 2:
        x = _width - 1 - x;
        break;
    }

    return x;
}

/**************************************************************************/
/*!
        Apply current rotation in the Y direction

        @return the Y value with current rotation applied
 */
/**************************************************************************/
int16_t Adafruit_RA8875_applyRotationY(int16_t y) {
    switch (_rotation) {
    case 2:
        y = _height - 1 - y;
        break;
    }

    return y + _voffset;
}

/**************************************************************************/
/*!
            Draws a single pixel at the specified location

            @param x         The 0-based x location
            @param y         The 0-base y location
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawPixel(int16_t x, int16_t y, uint16_t color) {
    x = Adafruit_RA8875_applyRotationX(x);
    y = Adafruit_RA8875_applyRotationY(y);

    Adafruit_RA8875_writeReg(RA8875_CURH0, x);
    Adafruit_RA8875_writeReg(RA8875_CURH1, x >> 8);
    Adafruit_RA8875_writeReg(RA8875_CURV0, y);
    Adafruit_RA8875_writeReg(RA8875_CURV1, y >> 8);
    Adafruit_RA8875_writeCommand(RA8875_MRWC);

    // TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    SPI.transfer(RA8875_DATAWRITE);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);
//    SPI.transfer(color >> 8);
	uint8_t pHigh = color >> 8;
	HAL_SPI_Transmit(&_DIS_HSPI, &pHigh, 1, HAL_MAX_DELAY);
//    SPI.transfer(color);
	uint8_t pLow = color&0xFF;
	HAL_SPI_Transmit(&_DIS_HSPI, &pLow, 1, HAL_MAX_DELAY);
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
 Draws a series of pixels at the specified location without the overhead

 @param p         An array of RGB565 color pixels
 @param num     The number of the pixels to draw
 @param x         The 0-based x location
 @param y         The 0-base y location
 */
/**************************************************************************/
void Adafruit_RA8875_drawPixels(uint16_t *p, uint32_t num, int16_t x,
                                                                 int16_t y) {
	Adafruit_RA8875_writeReg(RA8875_CURH0, x);
	Adafruit_RA8875_writeReg(RA8875_CURH1, x >> 8);
	Adafruit_RA8875_writeReg(RA8875_CURV0, y);
	Adafruit_RA8875_writeReg(RA8875_CURV1, y >> 8);
	Adafruit_RA8875_writeCommand(RA8875_MRWC);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit_DMA(&_DIS_HSPI, &data2, 1);
	    while (num--) {
	    	uint8_t p1 = *p >> 8;
	    	uint8_t p2 = *p & 0xFF;
	    	HAL_SPI_Transmit_DMA(&_DIS_HSPI, &p1, 1);
//	        SPI.transfer(*p >> 8);
//	        SPI.transfer(*p & 0xFF);
	        HAL_SPI_Transmit_DMA(&_DIS_HSPI, &p2, 1);
	        p++;
	    }
	    HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
            Draws a HW accelerated line on the display

            @param x0        The 0-based starting x location
            @param y0        The 0-base starting y location
            @param x1        The 0-based ending x location
            @param y1        The 0-base ending y location
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                                                             uint16_t color) {
    x0 = Adafruit_RA8875_applyRotationX(x0);
    y0 = Adafruit_RA8875_applyRotationY(y0);
    x1 = Adafruit_RA8875_applyRotationX(x1);
    y1 = Adafruit_RA8875_applyRotationY(y1);

    /* Set X */
    Adafruit_RA8875_writeCommand(0x91);
    Adafruit_RA8875_writeData(x0);
    Adafruit_RA8875_writeCommand(0x92);
    Adafruit_RA8875_writeData(x0 >> 8);

    /* Set Y */
    Adafruit_RA8875_writeCommand(0x93);
    Adafruit_RA8875_writeData(y0);
    Adafruit_RA8875_writeCommand(0x94);
    Adafruit_RA8875_writeData(y0 >> 8);

    /* Set X1 */
    Adafruit_RA8875_writeCommand(0x95);
    Adafruit_RA8875_writeData(x1);
    Adafruit_RA8875_writeCommand(0x96);
    Adafruit_RA8875_writeData((x1) >> 8);

    /* Set Y1 */
    Adafruit_RA8875_writeCommand(0x97);
    Adafruit_RA8875_writeData(y1);
    Adafruit_RA8875_writeCommand(0x98);
    Adafruit_RA8875_writeData((y1) >> 8);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(RA8875_DCR);
    Adafruit_RA8875_writeData(0x80);

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
        Draw a vertical line

        @param x The X position
        @param y The Y position
        @param h Height
        @param color The color
*/
/**************************************************************************/
void Adafruit_RA8875_drawFastVLine(int16_t x, int16_t y, int16_t h,
                                                                        uint16_t color) {
    Adafruit_RA8875_drawLine(x, y, x, y + h, color);
}

/**************************************************************************/
/*!
         Draw a horizontal line

         @param x The X position
         @param y The Y position
         @param w Width
         @param color The color
*/
/**************************************************************************/
void Adafruit_RA8875_drawFastHLine(int16_t x, int16_t y, int16_t w,
                                                                        uint16_t color) {
    Adafruit_RA8875_drawLine(x, y, x + w, y, color);
}

/**************************************************************************/
/*!
            Draws a HW accelerated rectangle on the display

            @param x         The 0-based x location of the top-right corner
            @param y         The 0-based y location of the top-right corner
            @param w         The rectangle width
            @param h         The rectangle height
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                             uint16_t color) {
    Adafruit_RA8875_rectHelper(x, y, x + w - 1, y + h - 1, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled rectangle on the display

            @param x         The 0-based x location of the top-right corner
            @param y         The 0-based y location of the top-right corner
            @param w         The rectangle width
            @param h         The rectangle height
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                             uint16_t color) {
    Adafruit_RA8875_rectHelper(x, y, x + w - 1, y + h - 1, color, true);
}

/**************************************************************************/
/*!
            Fills the screen with the spefied RGB565 color

            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_fillScreen(uint16_t color) {
    Adafruit_RA8875_rectHelper(0, 0, _width - 1, _height - 1, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated circle on the display

            @param x         The 0-based x location of the center of the circle
            @param y         The 0-based y location of the center of the circle
            @param r         The circle's radius
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawCircle(int16_t x, int16_t y, int16_t r,
                                                                 uint16_t color) {
    Adafruit_RA8875_circleHelper(x, y, r, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled circle on the display

            @param x         The 0-based x location of the center of the circle
            @param y         The 0-based y location of the center of the circle
            @param r         The circle's radius
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_fillCircle(int16_t x, int16_t y, int16_t r,
                                                                 uint16_t color) {
    Adafruit_RA8875_circleHelper(x, y, r, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated triangle on the display

            @param x0        The 0-based x location of point 0 on the triangle
            @param y0        The 0-based y location of point 0 on the triangle
            @param x1        The 0-based x location of point 1 on the triangle
            @param y1        The 0-based y location of point 1 on the triangle
            @param x2        The 0-based x location of point 2 on the triangle
            @param y2        The 0-based y location of point 2 on the triangle
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawTriangle(int16_t x0, int16_t y0, int16_t x1,
                                                                     int16_t y1, int16_t x2, int16_t y2,
                                                                     uint16_t color) {
    Adafruit_RA8875_triangleHelper(x0, y0, x1, y1, x2, y2, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled triangle on the display

            @param x0        The 0-based x location of point 0 on the triangle
            @param y0        The 0-based y location of point 0 on the triangle
            @param x1        The 0-based x location of point 1 on the triangle
            @param y1        The 0-based y location of point 1 on the triangle
            @param x2        The 0-based x location of point 2 on the triangle
            @param y2        The 0-based y location of point 2 on the triangle
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_fillTriangle(int16_t x0, int16_t y0, int16_t x1,
                                                                     int16_t y1, int16_t x2, int16_t y2,
                                                                     uint16_t color) {
    Adafruit_RA8875_triangleHelper(x0, y0, x1, y1, x2, y2, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated ellipse on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawEllipse(int16_t xCenter, int16_t yCenter,
                                                                    int16_t longAxis, int16_t shortAxis,
                                                                    uint16_t color) {
    Adafruit_RA8875_ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled ellipse on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_fillEllipse(int16_t xCenter, int16_t yCenter,
                                                                    int16_t longAxis, int16_t shortAxis,
                                                                    uint16_t color) {
    Adafruit_RA8875_ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated curve on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param curvePart The corner to draw, where in clock-wise motion:
                                                        0 = 180-270°
                                                        1 = 270-0°
                                                        2 = 0-90°
                                                        3 = 90-180°
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_drawCurve(int16_t xCenter, int16_t yCenter,
                                                                int16_t longAxis, int16_t shortAxis,
                                                                uint8_t curvePart, uint16_t color) {
    Adafruit_RA8875_curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled curve on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param curvePart The corner to draw, where in clock-wise motion:
                                                        0 = 180-270°
                                                        1 = 270-0°
                                                        2 = 0-90°
                                                        3 = 90-180°
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875_fillCurve(int16_t xCenter, int16_t yCenter,
                                                                int16_t longAxis, int16_t shortAxis,
                                                                uint8_t curvePart, uint16_t color) {
    Adafruit_RA8875_curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated rounded rectangle on the display

            @param x     The 0-based x location of the rectangle's upper left corner
            @param y     The 0-based y location of the rectangle's upper left corner
            @param w     The size in pixels of the rectangle's width
            @param h     The size in pixels of the rectangle's height
            @param r     The radius of the curves in the corners of the rectangle
            @param color    The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void Adafruit_RA8875_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                                        int16_t r, uint16_t color) {
    Adafruit_RA8875_roundRectHelper(x, y, x + w, y + h, r, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled rounded rectangle on the display

            @param x     The 0-based x location of the rectangle's upper left corner
            @param y     The 0-based y location of the rectangle's upper left corner
            @param w     The size in pixels of the rectangle's width
            @param h     The size in pixels of the rectangle's height
            @param r     The radius of the curves in the corners of the rectangle
            @param color    The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void Adafruit_RA8875_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                                        int16_t r, uint16_t color) {
    Adafruit_RA8875_roundRectHelper(x, y, x + w, y + h, r, color, true);
}

/**************************************************************************/
/*!
            Helper function for higher level circle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875_circleHelper(int16_t x, int16_t y, int16_t r,
                                                                     uint16_t color, bool filled) {
    x = Adafruit_RA8875_applyRotationX(x);
    y = Adafruit_RA8875_applyRotationY(y);

    /* Set X */
    Adafruit_RA8875_writeCommand(0x99);
    Adafruit_RA8875_writeData(x);
    Adafruit_RA8875_writeCommand(0x9a);
    Adafruit_RA8875_writeData(x >> 8);

    /* Set Y */
    Adafruit_RA8875_writeCommand(0x9b);
    Adafruit_RA8875_writeData(y);
    Adafruit_RA8875_writeCommand(0x9c);
    Adafruit_RA8875_writeData(y >> 8);

    /* Set Radius */
    Adafruit_RA8875_writeCommand(0x9d);
    Adafruit_RA8875_writeData(r);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(RA8875_DCR);
    if (filled) {
        Adafruit_RA8875_writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
    } else {
        Adafruit_RA8875_writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875_rectHelper(int16_t x, int16_t y, int16_t w, int16_t h,
                                                                 uint16_t color, bool filled) {
    x = Adafruit_RA8875_applyRotationX(x);
    y = Adafruit_RA8875_applyRotationY(y);
    w = Adafruit_RA8875_applyRotationX(w);
    h = Adafruit_RA8875_applyRotationY(h);

    /* Set X */
    Adafruit_RA8875_writeCommand(0x91);
    Adafruit_RA8875_writeData(x);
    Adafruit_RA8875_writeCommand(0x92);
    Adafruit_RA8875_writeData(x >> 8);

    /* Set Y */
    Adafruit_RA8875_writeCommand(0x93);
    Adafruit_RA8875_writeData(y);
    Adafruit_RA8875_writeCommand(0x94);
    Adafruit_RA8875_writeData(y >> 8);

    /* Set X1 */
    Adafruit_RA8875_writeCommand(0x95);
    Adafruit_RA8875_writeData(w);
    Adafruit_RA8875_writeCommand(0x96);
    Adafruit_RA8875_writeData((w) >> 8);

    /* Set Y1 */
    Adafruit_RA8875_writeCommand(0x97);
    Adafruit_RA8875_writeData(h);
    Adafruit_RA8875_writeCommand(0x98);
    Adafruit_RA8875_writeData((h) >> 8);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(RA8875_DCR);
    if (filled) {
        Adafruit_RA8875_writeData(0xB0);
    } else {
        Adafruit_RA8875_writeData(0x90);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level triangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875_triangleHelper(int16_t x0, int16_t y0, int16_t x1,
                                                                         int16_t y1, int16_t x2, int16_t y2,
                                                                         uint16_t color, bool filled) {
    x0 = Adafruit_RA8875_applyRotationX(x0);
    y0 = Adafruit_RA8875_applyRotationY(y0);
    x1 = Adafruit_RA8875_applyRotationX(x1);
    y1 = Adafruit_RA8875_applyRotationY(y1);
    x2 = Adafruit_RA8875_applyRotationX(x2);
    y2 = Adafruit_RA8875_applyRotationY(y2);

    /* Set Point 0 */
    Adafruit_RA8875_writeCommand(0x91);
    Adafruit_RA8875_writeData(x0);
    Adafruit_RA8875_writeCommand(0x92);
    Adafruit_RA8875_writeData(x0 >> 8);
    Adafruit_RA8875_writeCommand(0x93);
    Adafruit_RA8875_writeData(y0);
    Adafruit_RA8875_writeCommand(0x94);
    Adafruit_RA8875_writeData(y0 >> 8);

    /* Set Point 1 */
    Adafruit_RA8875_writeCommand(0x95);
    Adafruit_RA8875_writeData(x1);
    Adafruit_RA8875_writeCommand(0x96);
    Adafruit_RA8875_writeData(x1 >> 8);
    Adafruit_RA8875_writeCommand(0x97);
    Adafruit_RA8875_writeData(y1);
    Adafruit_RA8875_writeCommand(0x98);
    Adafruit_RA8875_writeData(y1 >> 8);

    /* Set Point 2 */
    Adafruit_RA8875_writeCommand(0xA9);
    Adafruit_RA8875_writeData(x2);
    Adafruit_RA8875_writeCommand(0xAA);
    Adafruit_RA8875_writeData(x2 >> 8);
    Adafruit_RA8875_writeCommand(0xAB);
    Adafruit_RA8875_writeData(y2);
    Adafruit_RA8875_writeCommand(0xAC);
    Adafruit_RA8875_writeData(y2 >> 8);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(RA8875_DCR);
    if (filled) {
        Adafruit_RA8875_writeData(0xA1);
    } else {
        Adafruit_RA8875_writeData(0x81);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level ellipse drawing code
*/
/**************************************************************************/
void Adafruit_RA8875_ellipseHelper(int16_t xCenter, int16_t yCenter,
                                                                        int16_t longAxis, int16_t shortAxis,
                                                                        uint16_t color, bool filled) {
    xCenter = Adafruit_RA8875_applyRotationX(xCenter);
    yCenter = Adafruit_RA8875_applyRotationY(yCenter);

    /* Set Center Point */
    Adafruit_RA8875_writeCommand(0xA5);
    Adafruit_RA8875_writeData(xCenter);
    Adafruit_RA8875_writeCommand(0xA6);
    Adafruit_RA8875_writeData(xCenter >> 8);
    Adafruit_RA8875_writeCommand(0xA7);
    Adafruit_RA8875_writeData(yCenter);
    Adafruit_RA8875_writeCommand(0xA8);
    Adafruit_RA8875_writeData(yCenter >> 8);

    /* Set Long and Short Axis */
    Adafruit_RA8875_writeCommand(0xA1);
    Adafruit_RA8875_writeData(longAxis);
    Adafruit_RA8875_writeCommand(0xA2);
    Adafruit_RA8875_writeData(longAxis >> 8);
    Adafruit_RA8875_writeCommand(0xA3);
    Adafruit_RA8875_writeData(shortAxis);
    Adafruit_RA8875_writeCommand(0xA4);
    Adafruit_RA8875_writeData(shortAxis >> 8);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(0xA0);
    if (filled) {
        Adafruit_RA8875_writeData(0xC0);
    } else {
        Adafruit_RA8875_writeData(0x80);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level curve drawing code
*/
/**************************************************************************/
void Adafruit_RA8875_curveHelper(int16_t xCenter, int16_t yCenter,
                                                                    int16_t longAxis, int16_t shortAxis,
                                                                    uint8_t curvePart, uint16_t color,
                                                                    bool filled) {
    xCenter = Adafruit_RA8875_applyRotationX(xCenter);
    yCenter = Adafruit_RA8875_applyRotationY(yCenter);
    curvePart = (curvePart + _rotation) % 4;

    /* Set Center Point */
    Adafruit_RA8875_writeCommand(0xA5);
    Adafruit_RA8875_writeData(xCenter);
    Adafruit_RA8875_writeCommand(0xA6);
    Adafruit_RA8875_writeData(xCenter >> 8);
    Adafruit_RA8875_writeCommand(0xA7);
    Adafruit_RA8875_writeData(yCenter);
    Adafruit_RA8875_writeCommand(0xA8);
    Adafruit_RA8875_writeData(yCenter >> 8);

    /* Set Long and Short Axis */
    Adafruit_RA8875_writeCommand(0xA1);
    Adafruit_RA8875_writeData(longAxis);
    Adafruit_RA8875_writeCommand(0xA2);
    Adafruit_RA8875_writeData(longAxis >> 8);
    Adafruit_RA8875_writeCommand(0xA3);
    Adafruit_RA8875_writeData(shortAxis);
    Adafruit_RA8875_writeCommand(0xA4);
    Adafruit_RA8875_writeData(shortAxis >> 8);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(0xA0);
    if (filled) {
        Adafruit_RA8875_writeData(0xD0 | (curvePart & 0x03));
    } else {
        Adafruit_RA8875_writeData(0x90 | (curvePart & 0x03));
    }

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level rounded rectangle drawing code
 */
/**************************************************************************/

void Adafruit_RA8875_swap(int16_t *x, int16_t *y) {
    int16_t temp = *x;
    *x = *y;
    *y = temp;
}

void Adafruit_RA8875_roundRectHelper(int16_t x, int16_t y, int16_t w,
                                                                            int16_t h, int16_t r, uint16_t color,
                                                                            bool filled) {
    x = Adafruit_RA8875_applyRotationX(x);
    y = Adafruit_RA8875_applyRotationY(y);
    w = Adafruit_RA8875_applyRotationX(w);
    h = Adafruit_RA8875_applyRotationY(h);
    if (x > w)
        Adafruit_RA8875_swap(x, w);
    if (y > h)
        Adafruit_RA8875_swap(y, h);

    /* Set X */
    Adafruit_RA8875_writeCommand(0x91);
    Adafruit_RA8875_writeData(x);
    Adafruit_RA8875_writeCommand(0x92);
    Adafruit_RA8875_writeData(x >> 8);

    /* Set Y */
    Adafruit_RA8875_writeCommand(0x93);
    Adafruit_RA8875_writeData(y);
    Adafruit_RA8875_writeCommand(0x94);
    Adafruit_RA8875_writeData(y >> 8);

    /* Set X1 */
    Adafruit_RA8875_writeCommand(0x95);
    Adafruit_RA8875_writeData(w);
    Adafruit_RA8875_writeCommand(0x96);
    Adafruit_RA8875_writeData((w) >> 8);

    /* Set Y1 */
    Adafruit_RA8875_writeCommand(0x97);
    Adafruit_RA8875_writeData(h);
    Adafruit_RA8875_writeCommand(0x98);
    Adafruit_RA8875_writeData((h) >> 8);

    Adafruit_RA8875_writeCommand(0xA1);
    Adafruit_RA8875_writeData(r);
    Adafruit_RA8875_writeCommand(0xA2);
    Adafruit_RA8875_writeData((r) >> 8);

    Adafruit_RA8875_writeCommand(0xA3);
    Adafruit_RA8875_writeData(r);
    Adafruit_RA8875_writeCommand(0xA4);
    Adafruit_RA8875_writeData((r) >> 8);

    /* Set Color */
    Adafruit_RA8875_writeCommand(0x63);
    Adafruit_RA8875_writeData((color & 0xf800) >> 11);
    Adafruit_RA8875_writeCommand(0x64);
    Adafruit_RA8875_writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875_writeCommand(0x65);
    Adafruit_RA8875_writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875_writeCommand(RA8875_ELLIPSE);
    if (filled) {
        Adafruit_RA8875_writeData(0xE0);
    } else {
        Adafruit_RA8875_writeData(0xA0);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875_waitPoll(RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}
/**************************************************************************/
/*!
            Set the scroll window

            @param x    X position of the scroll window
            @param y    Y position of the scroll window
            @param w    Width of the Scroll Window
            @param h    Height of the Scroll window
            @param mode Layer to Scroll

 */
/**************************************************************************/
void Adafruit_RA8875_setScrollWindow(int16_t x, int16_t y, int16_t w,
                                                                            int16_t h, uint8_t mode) {
    // Horizontal Start point of Scroll Window
    Adafruit_RA8875_writeCommand(0x38);
    Adafruit_RA8875_writeData(x);
    Adafruit_RA8875_writeCommand(0x39);
    Adafruit_RA8875_writeData(x >> 8);

    // Vertical Start Point of Scroll Window
    Adafruit_RA8875_writeCommand(0x3a);
    Adafruit_RA8875_writeData(y);
    Adafruit_RA8875_writeCommand(0x3b);
    Adafruit_RA8875_writeData(y >> 8);

    // Horizontal End Point of Scroll Window
    Adafruit_RA8875_writeCommand(0x3c);
    Adafruit_RA8875_writeData(x + w);
    Adafruit_RA8875_writeCommand(0x3d);
    Adafruit_RA8875_writeData((x + w) >> 8);

    // Vertical End Point of Scroll Window
    Adafruit_RA8875_writeCommand(0x3e);
    Adafruit_RA8875_writeData(y + h);
    Adafruit_RA8875_writeCommand(0x3f);
    Adafruit_RA8875_writeData((y + h) >> 8);

    // Scroll function setting
    Adafruit_RA8875_writeCommand(0x52);
    Adafruit_RA8875_writeData(mode);
}

/**************************************************************************/
/*!
        Scroll in the X direction

        @param dist The distance to scroll

 */
/**************************************************************************/
void Adafruit_RA8875_scrollX(int16_t dist) {
    Adafruit_RA8875_writeCommand(0x24);
    Adafruit_RA8875_writeData(dist);
    Adafruit_RA8875_writeCommand(0x25);
    Adafruit_RA8875_writeData(dist >> 8);
}

/**************************************************************************/
/*!
         Scroll in the Y direction

         @param dist The distance to scroll

 */
/**************************************************************************/
void Adafruit_RA8875_scrollY(int16_t dist) {
    Adafruit_RA8875_writeCommand(0x26);
    Adafruit_RA8875_writeData(dist);
    Adafruit_RA8875_writeCommand(0x27);
    Adafruit_RA8875_writeData(dist >> 8);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
        Set the Extra General Purpose IO Register

        @param on Whether to turn Extra General Purpose IO on or not

 */
/**************************************************************************/
void Adafruit_RA8875_GPIOX(bool on) {
    if (on)
        Adafruit_RA8875_writeReg(RA8875_GPIOX, 1);
    else
        Adafruit_RA8875_writeReg(RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!
        Set the duty cycle of the PWM 1 Clock

        @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void Adafruit_RA8875_PWM1out(uint8_t p) { Adafruit_RA8875_writeReg(RA8875_P1DCR, p); }

/**************************************************************************/
/*!
         Set the duty cycle of the PWM 2 Clock

         @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void Adafruit_RA8875_PWM2out(uint8_t p) { Adafruit_RA8875_writeReg(RA8875_P2DCR, p); }

/**************************************************************************/
/*!
        Configure the PWM 1 Clock

        @param on Whether to enable the clock
        @param clock The Clock Divider
*/
/**************************************************************************/
void Adafruit_RA8875_PWM1config(bool on, uint8_t clock) {
    if (on) {
        Adafruit_RA8875_writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
    } else {
        Adafruit_RA8875_writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
    }
}

/**************************************************************************/
/*!
         Configure the PWM 2 Clock

         @param on Whether to enable the clock
         @param clock The Clock Divider
*/
/**************************************************************************/
void Adafruit_RA8875_PWM2config(bool on, uint8_t clock) {
    if (on) {
        Adafruit_RA8875_writeReg(RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
    } else {
        Adafruit_RA8875_writeReg(RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
    }
}


/**************************************************************************/
/*!
            Checks if a touch event has occured

            @return    True is a touch event has occured (reading it via
                             touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
bool Adafruit_RA8875_touched(void) {
    if (Adafruit_RA8875_readReg(RA8875_INTC2) & RA8875_INTC2_TP)
        return true;
    return false;
}

/**************************************************************************/
/*!
            Reads the last touch event

            @param x    Pointer to the uint16_t field to assign the raw X value
            @param y    Pointer to the uint16_t field to assign the raw Y value

            @return True if successful

            @note Calling this function will clear the touch panel interrupt on
                        the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
bool Adafruit_RA8875_touchRead(uint16_t *x, uint16_t *y) {
    uint16_t tx, ty;
    uint8_t temp;

    tx = Adafruit_RA8875_readReg(RA8875_TPXH);
    ty = Adafruit_RA8875_readReg(RA8875_TPYH);
    temp = Adafruit_RA8875_readReg(RA8875_TPXYL);
    tx <<= 2;
    ty <<= 2;
    tx |= temp & 0x03;                // get the bottom x bits
    ty |= (temp >> 2) & 0x03; // get the bottom y bits

    *x = tx;
    *y = ty;

    /* Clear TP INT Status */
    Adafruit_RA8875_writeReg(RA8875_INTC2, RA8875_INTC2_TP);

    return true;
}

/**************************************************************************/
/*!
            Turns the display on or off

            @param on Whether to turn the display on or not
*/
/**************************************************************************/
void Adafruit_RA8875_displayOn(bool on) {
    if (on)
        Adafruit_RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
    else
        Adafruit_RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
        Puts the display in sleep mode, or disables sleep mode if enabled

        @param sleep Whether to sleep or not
*/
/**************************************************************************/
void Adafruit_RA8875_sleep(bool sleep) {
    if (sleep)
        Adafruit_RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
    else
        Adafruit_RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

/************************* Low Level ***********************************/

/**************************************************************************/
/*!
        Write data to the specified register

        @param reg Register to write to
        @param val Value to write
*/
/**************************************************************************/
void Adafruit_RA8875_writeReg(uint8_t reg, uint8_t val) {
    Adafruit_RA8875_writeCommand(reg);
    Adafruit_RA8875_writeData(val);
}

/**************************************************************************/
/*!
        Set the register to read from

        @param reg Register to read

        @return The value
*/
/**************************************************************************/
uint8_t Adafruit_RA8875_readReg(uint8_t reg) {
    Adafruit_RA8875_writeCommand(reg);
    return Adafruit_RA8875_readData();
}

/**************************************************************************/
/*!
        Write data to the current register

        @param d Data to write
*/
/**************************************************************************/
void Adafruit_RA8875_writeData(uint8_t d) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    spi_begin();
//    SPI.transfer(RA8875_DATAWRITE);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);
//    SPI.transfer(d);
	HAL_SPI_Transmit(&_DIS_HSPI, &d, 1, HAL_MAX_DELAY);
//    spi_end();
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}



/**************************************************************************/
/*!
        Read the data from the current register

        @return The Value
*/
/**************************************************************************/
uint8_t Adafruit_RA8875_readData(void) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    spi_begin();
//
//    SPI.transfer(RA8875_DATAREAD);
	 uint8_t data3 = 0x40;
	HAL_SPI_Transmit(&_DIS_HSPI, &data3, 1, HAL_MAX_DELAY);
//    uint8_t x = SPI.transfer(0x0);
	uint8_t data4 = 0x00;
	uint8_t x;
	HAL_SPI_TransmitReceive(&_DIS_HSPI, &data4, &x, 1, HAL_MAX_DELAY);
//    spi_end();
//
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
    return x;
}

/**************************************************************************/
/*!
        Write a command to the current register

        @param d The data to write as a command
 */
/**************************************************************************/
void Adafruit_RA8875_writeCommand(uint8_t d) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    spi_begin();
//
//    SPI.transfer(RA8875_CMDWRITE);
    // TODO: clean this up
    uint8_t data1 = 0x80;
	HAL_SPI_Transmit(&_DIS_HSPI, &data1, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&_DIS_HSPI, &d, 1, HAL_MAX_DELAY);
//    SPI.transfer(d);
//    spi_end();

//
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
        Read the status from the current register

        @return The value
 */
/**************************************************************************/
uint8_t Adafruit_RA8875_readStatus(void) {

	// TODO: implement SPI and write functions for this
	//    digitalWrite(_cs, LOW);
//    spi_begin();
//    SPI.transfer(RA8875_CMDREAD);
//    uint8_t x = SPI.transfer(0x0);
//    spi_end();
//
//    digitalWrite(_cs, HIGH);
//    return x;
	return 1;
}

static lv_color_t buf_1[16000]; //TODO: Chose a buffer size. DISPLAY_WIDTH * 10 is one suggestion.
static lv_color_t buf_2[16000];

// ----------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
/* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_SAI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USB_PCD_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_RF_Init();
  /* USER CODE BEGIN 2 */

  // TODO: cleanup, I moved this up
  // YOOOOOOOO, future JJ. The buttons use the same pins as some of the SPI2
  // stuff. Make sure they are not being initialize lol
  MX_APPE_Init();

/* USER CODE BEGIN 2 */

  printf("################ Start of Screen Demo ################%d\n", count);


  uint8_t dummy_rst = 0xFF;
  Adafruit_RA8875(dummy_rst, hspi1);
  bool a = Adafruit_RA8875_begin();

  Adafruit_RA8875_displayOn(true);
  Adafruit_RA8875_GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  Adafruit_RA8875_PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  Adafruit_RA8875_PWM1out(255);

  Adafruit_RA8875_fillScreen(RA8875_WHITE);

  //Initialise LVGL UI library
  lv_init();

  int WIDTH = 800;
  int HEIGHT = 480;

  lv_display_t * disp = lv_display_create(WIDTH, HEIGHT); /*Basic initialization with horizontal and vertical resolution in pixels*/
  lv_display_set_flush_cb(disp, my_flush_cb); /*Set a flush callback to draw to the display*/
  lv_display_set_buffers(disp, buf_1, buf_2, sizeof(buf_1), LV_DISPLAY_RENDER_MODE_PARTIAL); /*Set an initialized buffer*/

  // ------------------
  // Layout Stuff

  /*Create a container with ROW flex direction*/
  lv_obj_t* cont_row = lv_obj_create(lv_screen_active());
  lv_obj_set_size(cont_row, 800, 480);
  lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 5);
  lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);

  /*Create a container with COLUMN flex direction*/
  /*lv_obj_t* cont_col = lv_obj_create(lv_screen_active());
  lv_obj_set_size(cont_col, 200, 150);
  lv_obj_align_to(cont_col, cont_row, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
  lv_obj_set_flex_flow(cont_col, LV_FLEX_FLOW_COLUMN);*/

  static lv_style_t style_indic;

  lv_style_init(&style_indic);
  lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
  lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_GREEN));


  lv_obj_t* bar = lv_bar_create(cont_row);
  lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR);
  lv_obj_set_size(bar, 150, 350);
  lv_obj_center(bar);
  lv_bar_set_range(bar, 0, 100);
  lv_bar_set_value(bar, 100, LV_ANIM_OFF);
  lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);

  // ------

  static lv_style_t style;
  lv_style_init(&style);

  //lv_style_set_radius(&style, 5);
  //lv_style_set_bg_opa(&style, LV_OPA_COVER);
  //lv_style_set_bg_color(&style, lv_palette_lighten(LV_PALETTE_GREY, 2));
  //lv_style_set_border_width(&style, 2);
  //lv_style_set_border_color(&style, lv_color_black());
  //lv_style_set_pad_all(&style, 10);

  lv_style_set_text_color(&style, lv_color_black());
  lv_style_set_text_font(&style, &lv_font_montserrat_48);
  //lv_style_set_text_letter_space(&style, 5);
  //lv_style_set_text_line_space(&style, 20);
  //lv_style_set_text_decor(&style, LV_TEXT_DECOR_UNDERLINE);

  /*Create an object with the new style*/
  lv_obj_t* obj = lv_label_create(cont_row);
  lv_obj_add_style(obj, &style, 0);
  lv_obj_set_width(obj, 300);
  lv_label_set_text(obj, " 3-1-2024\n " LV_SYMBOL_BLUETOOTH " " LV_SYMBOL_BATTERY_3 " " LV_SYMBOL_VOLUME_MAX "\n100 Minutes\nRemaining\n\n2000 PSI\n15 LPM");
  lv_obj_center(obj);
  lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_ROW);

  /*Create a container with COLUMN flex direction*/
  lv_obj_t* cont_col = lv_obj_create(cont_row);
  lv_obj_set_size(cont_col, 300, 400);
  //lv_obj_align_to(cont_col, cont_row, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
  lv_obj_set_flex_flow(cont_col, LV_FLEX_FLOW_COLUMN);

  /*A base style*/
  static lv_style_t style_base;
  lv_style_init(&style_base);
  lv_style_set_bg_color(&style_base, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_border_color(&style_base, lv_palette_darken(LV_PALETTE_LIGHT_BLUE, 3));
  lv_style_set_border_width(&style_base, 2);
  lv_style_set_radius(&style_base, 10);
  lv_style_set_shadow_width(&style_base, 10);
  lv_style_set_shadow_offset_y(&style_base, 5);
  lv_style_set_shadow_opa(&style_base, LV_OPA_50);
  lv_style_set_text_color(&style_base, lv_color_white());
  lv_style_set_width(&style_base, 100);
  lv_style_set_height(&style_base, LV_SIZE_CONTENT);
  lv_style_set_text_font(&style_base, &lv_font_montserrat_48);

  /*Create an object with the base style only*/
  lv_obj_t* obj_base = lv_obj_create(cont_col);
  lv_obj_add_style(obj_base, &style_base, 0);
  lv_obj_align(obj_base, LV_ALIGN_LEFT_MID, 20, 0);

  lv_obj_t* label = lv_label_create(obj_base);
  lv_label_set_text(label, "Settings");
  lv_obj_set_size(obj_base, 250, 100);
  lv_obj_set_flex_flow(label, LV_FLEX_FLOW_COLUMN);

  /*Create an object with the base style only*/
  lv_obj_t* obj_base2 = lv_obj_create(cont_col);
  lv_obj_add_style(obj_base2, &style_base, 0);
  lv_obj_align(obj_base2, LV_ALIGN_LEFT_MID, 20, 0);

  lv_obj_t* label2 = lv_label_create(obj_base2);
  lv_label_set_text(label2, "Charting");
  lv_obj_set_size(obj_base2, 250, 100);
  lv_obj_set_flex_flow(label2, LV_FLEX_FLOW_COLUMN);

  lv_obj_set_flex_grow(cont_row, 0);
  lv_obj_set_flex_grow(bar, 0);
  lv_obj_set_flex_grow(obj, 0);
  lv_obj_set_flex_grow(cont_col, 0);
  lv_obj_set_flex_grow(label, 0);
  lv_obj_set_flex_grow(label2, 0);

  /* USER CODE END 2 */

/* Init code for STM32_WPAN */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* Infinite loop */
  while (1)
  {
    lv_bar_set_value(bar, 80, LV_ANIM_OFF);
    lv_timer_handler();
    HAL_Delay(5);
    lv_bar_set_value(bar, 50, LV_ANIM_OFF);
    lv_timer_handler();
	HAL_Delay(5);
	lv_bar_set_value(bar, 10, LV_ANIM_OFF);
	lv_timer_handler();
	HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 6;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_USBCLK
                              |RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10707DBC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

// TODO: determine if needed
/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
//static void MX_RNG_Init(void)
//{
//
//  /* USER CODE BEGIN RNG_Init 0 */
//
//  /* USER CODE END RNG_Init 0 */
//
//  /* USER CODE BEGIN RNG_Init 1 */
//
//  /* USER CODE END RNG_Init 1 */
//  hrng.Instance = RNG;
//  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
//  if (HAL_RNG_Init(&hrng) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RNG_Init 2 */
//
//  /* USER CODE END RNG_Init 2 */
//
//}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_RTC_Init(void)
//{
//
//  /* USER CODE BEGIN RTC_Init 0 */
//
//  /* USER CODE END RTC_Init 0 */
//
//  /* USER CODE BEGIN RTC_Init 1 */
//
//  /* USER CODE END RTC_Init 1 */
//
//  /** Initialize RTC Only
//  */
//  hrtc.Instance = RTC;
//  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
//  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
//  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Enable the WakeUp
//  */
//  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RTC_Init 2 */
//
//  /* USER CODE END RTC_Init 2 */
//
//}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

// TODO: determine if needed
/**
  * Enable DMA controller clock
  */
//static void MX_DMA_Init(void)
//{
//
//  /* DMA controller clock enable */
//  __HAL_RCC_DMAMUX1_CLK_ENABLE();
//  __HAL_RCC_DMA1_CLK_ENABLE();
//  __HAL_RCC_DMA2_CLK_ENABLE();
//
//  /* DMA interrupt init */
//  /* DMA1_Channel4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 15, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
//  /* DMA2_Channel4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 15, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
//
//}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PULSE_CHRG_SEL_Pin|TFT_RST_Pin|I2S_AMP_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_CS_Pin|LED_BLUE_Pin|LED_YELLOW_Pin|GPIO_PIN_11
                          |GAIN_3dB_Pin|Gain_6dB_Pin|GAIN_15dB_Pin|BOOST_EN_Pin
                          |GAUGE_IO_Pin|BATT_DETECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIS_CS_Pin|PULSE_CHRG_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Gain_12dB_GPIO_Port, Gain_12dB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PULSE_CHRG_SEL_Pin TFT_RST_Pin I2S_AMP_SD_Pin */
  GPIO_InitStruct.Pin = PULSE_CHRG_SEL_Pin|TFT_RST_Pin|I2S_AMP_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD__CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD__CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD__CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin LED_BLUE_Pin LED_YELLOW_Pin PB11
                           GAIN_3dB_Pin Gain_6dB_Pin GAIN_15dB_Pin BOOST_EN_Pin
                           GAUGE_IO_Pin BATT_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|LED_BLUE_Pin|LED_YELLOW_Pin|GPIO_PIN_11
                          |GAIN_3dB_Pin|Gain_6dB_Pin|GAIN_15dB_Pin|BOOST_EN_Pin
                          |GAUGE_IO_Pin|BATT_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GYRO_EXTI_Pin EXTI_ROT_SWT_Pin */
  GPIO_InitStruct.Pin = GYRO_EXTI_Pin|EXTI_ROT_SWT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIS_CS_Pin PULSE_CHRG_EN_Pin */
  GPIO_InitStruct.Pin = DIS_CS_Pin|PULSE_CHRG_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_WAIT_Pin */
  GPIO_InitStruct.Pin = TFT_WAIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TFT_WAIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Gain_12dB_Pin */
  GPIO_InitStruct.Pin = Gain_12dB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Gain_12dB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_BUTTON_1_Pin EXTI_BUTTON_2_Pin EXTI_BUTTON_3_Pin */
  GPIO_InitStruct.Pin = EXTI_BUTTON_1_Pin|EXTI_BUTTON_2_Pin|EXTI_BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI_PWR_SWT_STAT_Pin */
  GPIO_InitStruct.Pin = EXTI_PWR_SWT_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI_PWR_SWT_STAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI_PRESSURE_ALARM_Pin */
  GPIO_InitStruct.Pin = EXTI_PRESSURE_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI_PRESSURE_ALARM_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
