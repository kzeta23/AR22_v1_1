/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * fix to Change Range bug 240411
  * main240411_Final.c and main240411.c are same file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

#include "../../MyLib/SSD1322_OLED_lib_NB/SSD1322_HW_Driver.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/SSD1322_API.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/SSD1322_GFX.h"

#include "../../MyLib/SSD1322_OLED_lib_NB/Fonts/FreeSans8pt7b.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Fonts/FreeSansBold16pt7b.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Fonts/FreeSansBold18pt7b.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Fonts/FreeSansBold36pt7b.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Fonts/FreeSansBold42pt7b.h"

#include "../../MyLib/SSD1322_OLED_lib_NB/Bitmap/pat_i_mat.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Bitmap/krecik.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Bitmap/creeper.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Bitmap/stars_4bpp.h"
#include "../../MyLib/SSD1322_OLED_lib_NB/Bitmap/tom_and_jerry.h"

#include "../../MyLib/MOVING_AVERAGE/moving_average.h"
#include "../../MyLib/EEPROM_I2C/STM32_EEPROM_I2C.h"		//for 24C01 (I2C EEPROM)

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
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t SPI4_TX_completed_flag = 1;    //flag indicating finish of SPI transmission

volatile uint16_t gTimerCnt;

volatile uint32_t gmCountLow = 0;
volatile uint32_t gmCountHigh = 0;
uint32_t gmCountLow_sum = 0;
uint16_t sum_no_count = 0;
bool flag_no_count = 0;
#define NO_COUNT_TIME	300	// 300sec

//to use in moving average
float gmCountLowFiltered = 0.5F;
float gmCountLowFiltered_prev = 0.5F;

float gmCountLow_ref = 0;
float gmCountLow_ref_prev = 0;

float gmCountHighFiltered = 0.5F;
float gmCountHighFiltered_prev = 0.5F;

float gmCountHigh_ref = 0;
float gmCountHigh_ref_prev = 0;

FilterTypeDef filterStructLow;
FilterTypeDef filterStructHigh;

float alpha_lo = 0.2;						// ema parameter
float alpha_hi = 0.2;						// ema parameter
float alpha_lo_ref = 0.4;						// ema low ref parameter
float alpha_hi_ref = 0.4;						// ema high ref parameter

// Low-range adaptive-EMA tuning (background-stability vs response trade-off).
// At low background (~0.45 cps) raw counts are 0/1/2 per sec; that Poisson noise makes the
// fast ref-EMA jitter, keeping |ref-filtered| above 0 and pinning alpha_lo well above its
// floor -> the display wobbles. EMA_DEADBAND_LOW ignores divergence below this many cps
// (treat as noise) so alpha_lo can settle to its floor when truly stable; ALPHA_LO_MIN then
// sets how long it integrates there. A real dose step gives large divergence -> alpha_lo
// saturates at its 0.45 max, so step/alarm response is unaffected by these two knobs.
#define EMA_DEADBAND_LOW	0.08F		// cps; noise deadband on |ref-filtered| (0 = old behaviour)
#define ALPHA_LO_MIN		0.005F		// alpha_lo floor when stable (was 0.01; lower = longer integration)

//Conversion Factor
#define GM_LOW_CONV_FACTOR 	0.601F			//Conversion Factor LND7128 = 0.556
#define GM_HIGH_CONV_FACTOR 48.000F			//Conversion Factor LND71631	= 0.05mSv/h/cps,     240418 KEARI
#define DEFAULT_ALARM_THRESHOLD 10			//uSv/h, applied when EEPROM alarm data is invalid/unreadable

float conv_factor_low = GM_LOW_CONV_FACTOR;
float conv_factor_high = GM_HIGH_CONV_FACTOR;
uint8_t t_conv_factor_low = 0;
uint8_t t_conv_factor_high = 0;

float gmDoseLow = 0;
float gmDoseHigh = 0;
//#define TAULOW 		0.0001F
#define TAULOW 		0.0001F
#define TAUHIGH 	0.00001F

bool flag_movingAverage = 0;

//for SSD1322
uint8_t tx_buf[256 * 64 / 2];
char buffer[100];

//alarm
uint32_t alarmThreshold = 0;
int8_t set_alarm_data[5];
uint8_t set_alarm_shift = 0;
bool flag_alarm_ack = 0;
bool flag_alarm_on = 0;
uint16_t alarm_ack_timer = 0;

//display
bool flag_display_set = 0;
bool flag_display_set_blanking = 0;
bool flag_display_test = 0;
bool flag_display_cf = 0;
bool flag_display_cf_blanking = 0;
bool flag_display_cf_lo_hi = 0;

//range high & Low
#define RANGE_LOW 	0 // LOW Range
#define RANGE_HIGH 	1 // HIGH Range

#define RANGE_VAL_CENTER 		1000		//  uSv/h, high : 1 ~ 100mSv/h
#define RANGE_VAL_HIGH 		RANGE_VAL_CENTER + 200   // RANGE_VAL_CENTER + 20%
#define RANGE_VAL_LOW     	RANGE_VAL_CENTER - 200	 // RANGE_VAL_CENTER + 20%

uint16_t rangeStatus = RANGE_LOW; // if this is 0, it operate in Low range

//for M95010 EEPROM
extern uint8_t eeRxBuffer[EEPROM_BUFFER_SIZE];
extern uint8_t EEPROM_StatusByte;
uint8_t eeTxBuffer[EEPROM_BUFFER_SIZE];
#define ADDRESS_CF_LO 10
#define ADDRESS_CF_HI 11

//uart
uint8_t testBuffer[32] = "Program Start...\r\n";

//about switch buttons (written in HAL_GPIO_EXTI_Callback ISR -> volatile)
volatile bool flag_SW_SET = 0;
volatile bool flag_SW_SHIFT = 0;
volatile bool flag_SW_UP = 0;
volatile bool flag_SW_DOWN = 0;
volatile bool flag_SW_ACK = 0;

//about time tick (written in HAL_TIM_PeriodElapsedCallback ISR -> volatile)
volatile bool tick_0_1sec = 0;
volatile bool tick_0_2sec = 0;
volatile bool tick_0_5sec = 0;
volatile bool tick_1_0sec = 0;

volatile uint16_t current_avg_time = 0;		// incremented in TIM4 ISR, cleared in main

float gm_tau_low = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void buzzer_on(void);		// defined later; used by power_on_selftest()
void buzzer_off(void);

//SPI transmission finished interrupt callback
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	SSD1322_HW_drive_CS_high();
	SPI4_TX_completed_flag = 1;
}

//to use printf in SWV ITM Console
//int _write(int file, char *ptr, int len)
//{
//  int DataIdx;
//
//  for (DataIdx = 0; DataIdx < len; DataIdx++)
//  {
////    __io_putchar(*ptr++);
//    ITM_SendChar(*ptr++);
//  }
//  return len;
//}

/* Used for general interrupts */
int _write(int file, unsigned char *p, int len)
{
	// blocking TX (was _IT, which silently drops data when the UART is BUSY)
	HAL_UART_Transmit(&huart1, p, len, 100);	// printf -> USART1 (STLink VCP, 115200). BLE(USART2) not used.
	return len;
}//int

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void load_alarm_data()
{
	uint8_t ee_cnt = 0;
	bool valid = true;
	EepromOperations rd;

  EEPROM_I2C_INIT(&hi2c1);
	HAL_Delay(10);

	// alarm uses addr 0-4 only; read just those 5 bytes. Read twice for reliability.
	EEPROM_I2C_ReadBuffer(eeRxBuffer, 0, 5);	//load alarm data (addr 0-4)
	HAL_Delay(10);
	rd = EEPROM_I2C_ReadBuffer(eeRxBuffer, 0, 5);	//load alarm data (addr 0-4)
	HAL_Delay(10);

	// validate : read must succeed and every digit must be 0~9
	if(rd != EEPROM_STATUS_COMPLETE) valid = false;
	for(ee_cnt = 0; ee_cnt < 5; ee_cnt++)
	{
		if(eeRxBuffer[ee_cnt] > 9) valid = false;
	}//for

	if(valid)											// apply alarm data from Rx Buffer
	{
		for(ee_cnt = 0; ee_cnt < 5; ee_cnt++)
			set_alarm_data[ee_cnt] = eeRxBuffer[ee_cnt];
	}//if
	else												// EEPROM invalid/unreadable -> apply default (DEFAULT_ALARM_THRESHOLD)
	{
		set_alarm_data[0] = (DEFAULT_ALARM_THRESHOLD / 10000) % 10;
		set_alarm_data[1] = (DEFAULT_ALARM_THRESHOLD / 1000) % 10;
		set_alarm_data[2] = (DEFAULT_ALARM_THRESHOLD / 100) % 10;
		set_alarm_data[3] = (DEFAULT_ALARM_THRESHOLD / 10) % 10;
		set_alarm_data[4] = (DEFAULT_ALARM_THRESHOLD) % 10;
	}//else

	alarmThreshold =														// calculate alarm Threshold data
			set_alarm_data[0] * 10000 +
			set_alarm_data[1] * 1000 +
			set_alarm_data[2] * 100 +
			set_alarm_data[3] * 10 +
			set_alarm_data[4];

	HAL_Delay(10);
//	printf("Load Data = %d%d%d%d%d\r\n", eeRxBuffer[0], eeRxBuffer[1], eeRxBuffer[2], eeRxBuffer[3], eeRxBuffer[4]);
	HAL_Delay(10);
//	printf("Alarm Threshold = %05lu\r\n", alarmThreshold);

}//void

void save_alarm_data()
{
	uint8_t ee_cnt = 0;

  EEPROM_I2C_INIT(&hi2c1);
	HAL_Delay(10);

	for(ee_cnt = 0; ee_cnt < 5; ee_cnt++)				// apply alarm data to Tx
	{
		eeTxBuffer[ee_cnt] = set_alarm_data[ee_cnt];
	}//for

	alarmThreshold =														// calculate alarm Threshold data
			set_alarm_data[0] * 10000 +
			set_alarm_data[1] * 1000 +
			set_alarm_data[2] * 100 +
			set_alarm_data[3] * 10 +
			set_alarm_data[4];

	// alarm uses addr 0-4 only; write just those 5 bytes (24C01 byte/partial write).
	// addr 0-4 are within page 0, so a single 5-byte write is fine. Twice for reliability.
	EEPROM_I2C_WriteBuffer(eeTxBuffer, 0, 5);	//write alarm data (addr 0-4)
	HAL_Delay(10);
	EEPROM_I2C_WriteBuffer(eeTxBuffer, 0, 5);	//write alarm data (addr 0-4)
	HAL_Delay(10);

}//void

void load_cf_data()
{
	EepromOperations rd;

  EEPROM_I2C_INIT(&hi2c1);
	HAL_Delay(10);

	// c/f uses addr 26,27 only; read just those 2 bytes into eeRxBuffer[10],[11]
	// (= ADDRESS_CF_LO,_HI), keeping the indexing below unchanged. Read twice for reliability.
	EEPROM_I2C_ReadBuffer(&eeRxBuffer[ADDRESS_CF_LO], 16 + ADDRESS_CF_LO, 2);	//load c/f data (addr 26,27)
	HAL_Delay(10);
	rd = EEPROM_I2C_ReadBuffer(&eeRxBuffer[ADDRESS_CF_LO], 16 + ADDRESS_CF_LO, 2);	//load c/f data (addr 26,27)
	HAL_Delay(10);

	t_conv_factor_low  =	eeRxBuffer[ADDRESS_CF_LO] ;		// apply cf data from Rx Buffer & calculate cf low data
	t_conv_factor_high =	eeRxBuffer[ADDRESS_CF_HI] ;				// apply cf data from Rx Buffer &calculate cf low data

	conv_factor_low  =  (float)t_conv_factor_low  * 0.01;	// 60 -> 0.60
	conv_factor_high =  (float)t_conv_factor_high * 1;			// 48 -> 48

	if(rd != EEPROM_STATUS_COMPLETE || conv_factor_low < 0.4 || conv_factor_low > 0.8)
	{
		conv_factor_low = GM_LOW_CONV_FACTOR;		 // apply defualt (read fail or out of range)
	}//if

	if(rd != EEPROM_STATUS_COMPLETE || conv_factor_high < 34 || conv_factor_high > 66)
	{
		conv_factor_high = GM_HIGH_CONV_FACTOR;		// apply defualt (read fail or out of range)
	}//if

	HAL_Delay(10);
}//void

void save_cf_data()
{
  EEPROM_I2C_INIT(&hi2c1);
	HAL_Delay(10);

//	t_conv_factor_low  = (uint8_t)(conv_factor_low  * 100);		  // 0.6 -> 60
//	t_conv_factor_high = (uint8_t)(conv_factor_high * 1);		  	// 48 -> 48

	eeTxBuffer[ADDRESS_CF_LO] = t_conv_factor_low;		  	// 0.6 -> 60
	eeTxBuffer[ADDRESS_CF_HI] = t_conv_factor_high;		  	// 48 -> 48

	conv_factor_low  =  (float)t_conv_factor_low  * 0.01;	// 60 -> 0.60
	conv_factor_high =  (float)t_conv_factor_high * 1;			// 48 -> 48

	if(conv_factor_low < 0.4 || conv_factor_low > 0.8)
	{
		conv_factor_low = GM_LOW_CONV_FACTOR;		 // apply defualt
	}//if

	if(conv_factor_high < 34 || conv_factor_high > 66)
	{
		conv_factor_high = GM_HIGH_CONV_FACTOR;		// apply defualt
	}//if

	// 24C01 supports byte/partial writes, so write only the 2 c/f bytes (EEPROM addr 26,27)
	// instead of the whole 16-byte block. Avoids wearing/clobbering unused cells and
	// stops stale eeTxBuffer contents from being written to addr 16-25,28-31.
	// addr 26,27 are in the same 8-byte page (24-31), so a single 2-byte write is fine.
	// Written twice for reliability (matches the read-twice in load_cf_data).
	EEPROM_I2C_WriteBuffer(&eeTxBuffer[ADDRESS_CF_LO], 16 + ADDRESS_CF_LO, 2);	//write c/f data (addr 26,27)
	HAL_Delay(10);
	EEPROM_I2C_WriteBuffer(&eeTxBuffer[ADDRESS_CF_LO], 16 + ADDRESS_CF_LO, 2);	//write c/f data (addr 26,27)
	HAL_Delay(10);

}//void

void init_system()
{
	//Removed GM_SW_HI / GM_SW_LO / LAMP_GREEN : not assigned in current .ioc
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);		//Buzzer off (ensure TIM5_CH2 PWM stopped)
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);	//LAMP_RED off

	__HAL_TIM_SetCounter(&htim1, 0);	//clear
	__HAL_TIM_SetCounter(&htim8, 0);	//clear

	//init moving average
	Moving_Average_Init_Low(&filterStructLow);
	Moving_Average_Init_High(&filterStructHigh);
}//void

void check_display_monitor()
{
	if(HAL_GPIO_ReadPin(SW_SHIFT_GPIO_Port, SW_SHIFT_Pin) == 0)		// if push shift swith
	{
		while(HAL_GPIO_ReadPin(SW_SHIFT_GPIO_Port, SW_SHIFT_Pin) == 0)
		{
			sprintf (buffer, "Data Monitor");
			draw_text(tx_buf, buffer, 0, 56, 15);

			// send a frame buffer to the display
			send_buffer_to_OLED(tx_buf, 0, 0);
		}//while
	    flag_display_test = 1;	// enter display test mode
	    flag_SW_SHIFT = 0;		// do not act shift switch, set default '0'
	}//if
}//void

void check_setup_cf()
{
	if(HAL_GPIO_ReadPin(SW_SET_GPIO_Port, SW_SET_Pin) == 0)		// if push set swith
	{
		while(HAL_GPIO_ReadPin(SW_SET_GPIO_Port, SW_SET_Pin) == 0)
		{
			sprintf (buffer, "Setup C/F");
			draw_text(tx_buf, buffer, 0, 56, 15);

			// send a frame buffer to the display
			send_buffer_to_OLED(tx_buf, 0, 0);
		}//while

	  flag_display_cf = 1;	// enter display test mode
	  flag_SW_SET = 0;		// do not act shift switch, set default '0'

		load_cf_data();				// load cf data from external eeprom
//	    t_conv_factor_low  = (uint8_t)(conv_factor_low  * 100);
//	    t_conv_factor_high = (uint8_t)(conv_factor_high * 1);
	}//if
}//void

void display_firm_ver()
{
	char *fileName = __FILE_NAME__;
	char *compileDate = __DATE__;

	select_font(&FreeSans8pt7b);		// select font

	//display file information
	sprintf (buffer, "FirmWare [%s]", fileName);
	draw_text(tx_buf, buffer, 0, 14, 15);
	sprintf (buffer, "Compile [%s]", compileDate);
	draw_text(tx_buf, buffer, 0, 28, 15);

	// send a frame buffer to the display
	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void init_display()
{
	//Call initialization sequence for SSD1322
	SSD1322_API_init();

	// Lower panel drive current for OLED longevity (0xC1: 0xFF max -> 0x80 half).
	// Halving the drive current markedly slows luminance decay / burn-in.
	SSD1322_API_set_contrast(0x80);

	//Set frame buffer size in pixels - it is used to avoid writing to memory outside frame buffer
	//Normally it has to only be done once on initialization, but buffer size is changed near the end of while(1);.
	set_buffer_size(256, 64);
	// Fill buffer with zeros to clear any garbage values
	fill_buffer(tx_buf, 0);

	// send a frame buffer to the display
	send_buffer_to_OLED(tx_buf, 0, 0);
	HAL_Delay(1000);

	display_firm_ver();

	check_display_monitor();

	check_setup_cf();

	HAL_Delay(1000);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);	// enable SW interrupt
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);		// enable SW interrupt

}//void

// Power-On Self Test : exercise display, LEDs, buzzer and EEPROM so the operator
// can visually/audibly check for faults at startup. Call after init_display().
void power_on_selftest()
{
	uint8_t i;
	bool ee_ok = true;

	// 1) Display : full-screen ON to reveal dead pixels/lines or SPI faults
	fill_buffer(tx_buf, 15);								// all pixels at max brightness
	send_buffer_to_OLED(tx_buf, 0, 0);
	HAL_Delay(1000);

	// status screen (lines accumulate as each step runs)
	fill_buffer(tx_buf, 0);
	select_font(&FreeSans8pt7b);
	draw_text(tx_buf, "SELF TEST", 0, 14, 15);
	draw_text(tx_buf, "Display ... OK", 0, 28, 15);
	send_buffer_to_OLED(tx_buf, 0, 0);
	HAL_Delay(500);

	// 2) LED : blink LED1 + LED2 (x3), then LAMP_RED (x3)
	for(i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
	}//for
	for(i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
	}//for
	draw_text(tx_buf, "LED / LAMP ... OK", 0, 42, 15);
	send_buffer_to_OLED(tx_buf, 0, 0);

	// 3) Buzzer : 2 short beeps (TIM5_CH2 PWM)
	for(i = 0; i < 2; i++)
	{
		buzzer_on();
		HAL_Delay(200);
		buzzer_off();
		HAL_Delay(200);
	}//for
	draw_text(tx_buf, "Buzzer OK", 0, 56, 15);
	send_buffer_to_OLED(tx_buf, 0, 0);

	// 4) EEPROM (24C01 / I2C) : presence + write-read-verify on an UNUSED cell,
	//    then restore the original byte. Real data (alarm@0-4, c/f@26,27) untouched.
	if(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDR, 3, 100) != HAL_OK)
	{
		ee_ok = false;									// no ACK : I2C bus / device fault
	}
	else
	{
		uint8_t scratch_addr = 0x7F;					// 24C01 last address (127), unused
		uint8_t orig = 0, test = 0xA5, rd = 0;

		EEPROM_I2C_INIT(&hi2c1);
		EEPROM_I2C_ReadBuffer(&orig, scratch_addr, 1);	// keep original value
		if(EEPROM_I2C_WriteBuffer(&test, scratch_addr, 1) != EEPROM_STATUS_COMPLETE)
			ee_ok = false;
		EEPROM_I2C_ReadBuffer(&rd, scratch_addr, 1);
		if(rd != test)
			ee_ok = false;								// read-back mismatch
		EEPROM_I2C_WriteBuffer(&orig, scratch_addr, 1);	// restore original byte
	}

	draw_text(tx_buf, ee_ok ? "EEPROM OK" : "EEPROM FAIL", 140, 56, 15);
	send_buffer_to_OLED(tx_buf, 0, 0);

	// on EEPROM fault, sound 3 long beeps as a warning but keep running
	if(!ee_ok)
	{
		for(i = 0; i < 3; i++)
		{
			buzzer_on();
			HAL_Delay(400);
			buzzer_off();
			HAL_Delay(150);
		}//for
	}//if

	HAL_Delay(1500);									// hold result screen for the operator

	// 5) Restore : all indicators off, clear display
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);
	buzzer_off();
	fill_buffer(tx_buf, 0);
	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void display_alarm()
{
	uint16_t index_init = 165;
	uint16_t index_add = 10;
	uint8_t index_cnt = 0;

	select_font(&FreeSans8pt7b);

	// normal display mode
	switch(rangeStatus)
	{
	case RANGE_LOW:
		draw_text(tx_buf, "Alarm", 165, 14, 7);		//display 'alarm' when in low range
		break;
	case RANGE_HIGH:
		draw_text(tx_buf, "ALARM", 165, 14, 7);		//display 'ALARM' when in low range
		break;
	}//switch

	for(index_cnt=0; index_cnt<5; index_cnt++)	// display set data
	{
		sprintf(buffer, "%d", set_alarm_data[index_cnt]);
		draw_text(tx_buf, buffer, index_init+index_add*index_cnt, 28, 7);
	}//for

	// setting display mode
	if(flag_display_set == 1) 						// enable setting mode
	{
		if((flag_display_set_blanking ^= 1) == 1)	// flashing select position digit
		{
			sprintf(buffer, "%d", set_alarm_data[set_alarm_shift]);
			draw_text(tx_buf, buffer, index_init+index_add*set_alarm_shift, 28, 3);
		}//if
	}//if

	draw_text(tx_buf, "uSv/h", 215, 28, 7);

	// send a frame buffer to the display
	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void display_dose_low()
{
	// Fill buffer with zeros to clear any garbage values
	fill_buffer(tx_buf, 0);

	if(flag_no_count == 1)									// No count for 5min
	{
		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "NO COUNT FAIL", 0, 60, 15);
	}//if

	else if(gmDoseLow >= 100000.000F )			// >= 100mSv/h, 999mSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.0f", gmDoseLow/1000);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "mSv/h", 162, 60, 15);
	}//if

	else if(gmDoseLow >= 10000.000F )			// >= 10mSv/h, 99.9mSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.1f", gmDoseLow/1000);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "mSv/h", 162, 60, 15);
	}//if

	else if(gmDoseLow >= 100.000F ) 		// >= 0.1mSv/h, 9.99mSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.2f", gmDoseLow/1000);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "mSv/h", 162, 60, 15);
	}//else if

	else if(gmDoseLow >= 10.000F )		// >= 10uSv/h, 99.9uSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.1f", gmDoseLow);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "uSv/h", 162, 60, 15);
	}//else if

	else if(gmDoseLow >= 0.000F )// >= 0uSv/h, 9.99uSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.2f", gmDoseLow);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "uSv/h", 162, 60, 15);
	}

	// send a frame buffer to the display
//	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void display_dose_high()
{
	// Fill buffer with zeros to clear any garbage values
	fill_buffer(tx_buf, 0);

	if(gmDoseHigh >= 150000.000F )			// >= 100mSv/h, 999mSv/h
	{
		select_font(&FreeSansBold18pt7b);
		draw_text(tx_buf, "OVERLOAD", 28, 60, 15);
	}//if

	else if(gmDoseHigh >= 100000.000F )			// >= 100mSv/h, 999mSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.0f", gmDoseHigh/1000);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "mSv/h", 162, 60, 15);
	}//if

	else if(gmDoseHigh >= 10000.000F )			// >= 10mSv/h, 99.9mSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.1f", gmDoseHigh/1000);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "mSv/h", 162, 60, 15);
	}//if

	else if(gmDoseHigh >= 100.000F ) 		// >= 0.1mSv/h, 9.99mSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.2f", gmDoseHigh/1000);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "mSv/h", 162, 60, 15);
	}//else if

	else if(gmDoseHigh >= 10.000F )		// >= 10uSv/h, 99.9uSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.1f", gmDoseHigh);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "uSv/h", 162, 60, 15);
	}//else if

	else if(gmDoseHigh >= 0.000F )// >= 0uSv/h, 9.99uSv/h
	{
		select_font(&FreeSansBold42pt7b);
		sprintf (buffer, "%.2f", gmDoseHigh);
		draw_text(tx_buf, buffer, 0, 60, 15);

		select_font(&FreeSansBold16pt7b);
		draw_text(tx_buf, "uSv/h", 162, 60, 15);
	}

	// send a frame buffer to the display
//	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void display_dose()	// display Radiation value
{
	switch(rangeStatus)
	{
	case RANGE_LOW:
		display_dose_low();
		break;
	case RANGE_HIGH:
		display_dose_high();
		break;
	}//switch
	display_alarm();
}//void

// NOTE: side-effect + LOW-range only. Besides detecting "no count", this also
//       clamps gmDoseLow to a 0.05 floor (minimum displayed dose). It inspects
//       only gmDoseLow; the NO COUNT FAIL flag is shown in the LOW-range display
//       and data monitor, NOT in the HIGH-range display.
void check_no_count()
{
	if(gmDoseLow < 0.05F)		// if below 0.05, display minimum dose (floor)
	{
		gmDoseLow = 0.05F;

		if(sum_no_count < NO_COUNT_TIME)	// every 5min(300sec)
		{
			sum_no_count++;				// increase count
			flag_no_count = 0;		// reset no count
		}//if
		else
		{
			flag_no_count = 1;		// set no count
		}//else
	}//if

	else										// if over 0.15, reset no count
	{
		sum_no_count = 0;			// clear
		flag_no_count = 0;		// set no count
	}//else
}//void

void display_monitor()	// display Radiation value
{
	// Fill buffer with zeros to clear any garbage values
	fill_buffer(tx_buf, 0);

	select_font(&FreeSans8pt7b);		// select font

//	sprintf (buffer, "a/t:%02d", alarm_ack_timer);
//	draw_text(tx_buf, buffer, 210, 14, 15);
	if(flag_no_count == 1) sprintf (buffer, "FAIL");
	else                   sprintf (buffer, "NC:%02d", sum_no_count);
	draw_text(tx_buf, buffer, 205, 14, 15);

	sprintf (buffer, "[Lo] ar:%.3f af:%.3f", alpha_lo, alpha_lo_ref);
	draw_text(tx_buf, buffer, 0, 14, 15);

	sprintf (buffer, "[Hi] ar:%.3f af:%.3f", alpha_hi, alpha_hi_ref);
	draw_text(tx_buf, buffer, 0, 28, 15);

	if(gmCountLow < 100)
		sprintf (buffer, "C%04lu Ar%.2f Af%.2f D%.2f  ", gmCountLow, gmCountLow_ref, gmCountLowFiltered, gmDoseLow);
	else
		sprintf (buffer, "C%04lu Ar%.0f Af%.0f D%.0f  ", gmCountLow, gmCountLow_ref, gmCountLowFiltered, gmDoseLow);
	draw_text(tx_buf, buffer, 0, 42, 15);

	if(gmCountHigh < 100)
		sprintf (buffer, "C%04lu Ar%.2f Af%.2f D%.2f  ", gmCountHigh,  gmCountHigh_ref, gmCountHighFiltered, gmDoseHigh);
	else
		sprintf (buffer, "C%04lu Ar%.0f Af%.0f D%.0f  ", gmCountHigh,  gmCountHigh_ref, gmCountHighFiltered, gmDoseHigh);
	draw_text(tx_buf, buffer, 0, 56, 15);

	sprintf (buffer, "%1.2f  ", conv_factor_low);
	draw_text(tx_buf, buffer, 225, 42, 15);

	sprintf (buffer, "%2.1f  ", conv_factor_high);
	draw_text(tx_buf, buffer, 225, 56, 15);

	// send a frame buffer to the display
	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void display_cf()	// display Radiation value
{
	// Fill buffer with zeros to clear any garbage values
	fill_buffer(tx_buf, 0);

	select_font(&FreeSans8pt7b);		// select font

	sprintf (buffer, "[c/f lo]");
	draw_text(tx_buf, buffer, 0, 14, 15);

	sprintf (buffer, "[c/f Hi]");
	draw_text(tx_buf, buffer, 0, 28, 15);

	// setting cf display
	if(flag_display_cf == 1) 						// enable setting mode
	{
		if(flag_display_cf_lo_hi == 0)		// select cf low
		{
			if((flag_display_cf_blanking ^= 1) == 1) // flashing select position digit
			{
				sprintf (buffer, "0.%2d", t_conv_factor_low);	// display 0.%2d -> 0.60
				draw_text(tx_buf, buffer, 50, 14, 15);
			}//if
			sprintf (buffer, "%2d.0", t_conv_factor_high);	// not flashing
			draw_text(tx_buf, buffer, 50, 28, 15);
		}//if
		else															// select cf high
		{
			if((flag_display_cf_blanking ^= 1) == 1) // flashing select position digit
			{
				sprintf (buffer, "%2d.0", t_conv_factor_high);
				draw_text(tx_buf, buffer, 50, 28, 15);
			}//if
			sprintf (buffer, "0.%2d", t_conv_factor_low);	// not flashing
			draw_text(tx_buf, buffer, 50, 14, 15);
		}//else
		sprintf (buffer, "(ref:0.60)");														// display lo reference
		draw_text(tx_buf, buffer, 90, 14, 15);
		sprintf (buffer, "(ref:48.0)");															// display hi reference
		draw_text(tx_buf, buffer, 90, 28, 15);
	}//if

	// send a frame buffer to the display
	send_buffer_to_OLED(tx_buf, 0, 0);
}//void

void process_switch()
{
	if(flag_SW_SET == 1)
	{
		flag_SW_SET = 0;	//clear
		current_avg_time = 0;	//clear

		if(flag_display_set == 0)		//enter setting mode
		{
			flag_display_set = 1;
			load_alarm_data();				// load alarm data from external eeprom
		}//if
		else												//exit setting mode
		{
			flag_display_set = 0;
			save_alarm_data();				// save alarm data to external eeprom
		}//else

		if(flag_display_cf == 1)		//enter cf setting mode
		{
			flag_SW_SET = 0;	//clear
			flag_display_cf = 0;		// clear
			flag_display_set = 0;		// clear, prevent entry into setup mode
			save_cf_data();				// save cf data to external eeprom
		}//if

	}//if

	if(flag_SW_SHIFT == 1)
	{
		flag_SW_SHIFT = 0;	//clear

		if(flag_display_set == 1)		//when setting mode
		{
			set_alarm_shift++;
			if(set_alarm_shift > 4)
			{
				set_alarm_shift = 0;
			}//if
		}//if
		else
		{
//			flag_display_test ^= 1;		// toggle
		    flag_display_test = 0;		// exit display test mode
		}//else//¿

		if(flag_display_cf == 1)		// when setting conversion factor
		{
			flag_display_cf_lo_hi ^= 1; // toggle
		}
	}//if

	if(flag_SW_UP == 1)
	{
		flag_SW_UP = 0;	//clear

		if(flag_display_set == 1)		//when setting mode
		{
			set_alarm_data[set_alarm_shift]++;
			if(set_alarm_data[set_alarm_shift] > 9)
			{
				set_alarm_data[set_alarm_shift] = 0;
			}//if
		}//if

		if(flag_display_cf == 1)		// when setting conversion factor
		{
			if(flag_display_cf_lo_hi == 0)
			{
				if(t_conv_factor_low++ >= 99)
				{
					t_conv_factor_low = 0;
				}//if
			}//if
			else
			{
				if(t_conv_factor_high++ >= 99)
				{
					t_conv_factor_high = 0;
				}//if
			}//else
		}//if

	}//if

	if(flag_SW_DOWN == 1)
	{
		flag_SW_DOWN = 0;	//clear

		if(flag_display_set == 1)		//when setting mode
		{
			set_alarm_data[set_alarm_shift]--;
			if(set_alarm_data[set_alarm_shift] < 0)
			{
				set_alarm_data[set_alarm_shift] = 9;
			}//if
		}//if

		if(flag_display_cf == 1)		// when setting conversion factor
		{
			if(flag_display_cf_lo_hi == 0)
			{
				if(t_conv_factor_low-- <= 0)
				{
					t_conv_factor_low = 99;
				}//if
			}//if
			else
			{
				if(t_conv_factor_high-- <= 0)
				{
					t_conv_factor_high = 99;
				}//if
			}//else
		}//if

	}//if

	if(flag_SW_ACK == 1)
	{
		flag_SW_ACK = 0;
		flag_alarm_ack = 1;	//press the ACK button, Acknowledge enable for setting time
	}//

}//void

void lamp_safe()
{
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_RESET);					// RED LAMP OFF
		// LAMP_GREEN removed : not assigned in current .ioc
}//void

void lamp_alarm()
{
		HAL_GPIO_WritePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin, GPIO_PIN_SET);					// RED LAMP ON
		// LAMP_GREEN removed : not assigned in current .ioc
}//void

static uint8_t buzzer_active = 0;			// guard: avoid redundant PWM start/stop

void buzzer_on()
{
	if(!buzzer_active)
	{
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);								// BUZZER ON  (TIM5_CH2 PWM tone on PA1)
		buzzer_active = 1;
	}//if
}//void

void buzzer_off()
{
	if(buzzer_active)
	{
		HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);								// BUZZER OFF (stop TIM5_CH2 PWM)
		buzzer_active = 0;
	}//if
}//void

void process_alarm() // Decision to activate the alarm
{
	/* Alarm while Low Range */
	if(rangeStatus == RANGE_LOW && gmDoseLow >= alarmThreshold)		//Current dose_rate is over set alarmThreshold
	{
		if(flag_alarm_ack == 0)		//do not press ack button
		{
			buzzer_on();																							// BUZZER ON
	 		lamp_alarm();																								// red lamp on, green lamp off
		}//if

		else											//pressed ack button, disable buzzer for 60s
		{
			buzzer_off();																							// BUZZER OFF
	 		lamp_alarm();																								// red lamp on, green lamp off

			if(alarm_ack_timer++ >= 60)		// for 60sec
			{
				alarm_ack_timer = 0; 					//clear
				flag_alarm_ack = 0;						//claer
			}//if
		}//else

	}//if

	/* Alarm while High Range */
	else if(rangeStatus == RANGE_HIGH && gmDoseHigh >= alarmThreshold)		//Current dose_rate is over set alarmThreshold
	{
		if(flag_alarm_ack == 0)		//do not press ack button
		{
			buzzer_on();																							// BUZZER ON
	 		lamp_alarm();																								// red lamp on, green lamp off
		}//if

		else											//pressed ack button, disable buzzer for 60s
		{
			buzzer_off();																							// BUZZER OFF
	 		lamp_alarm();																								// red lamp on, green lamp off

			if(alarm_ack_timer++ >= 60)		// for 60sec
			{
				alarm_ack_timer = 0; 					//clear
				flag_alarm_ack = 0;						//claer
			}//if
		}//else

	}//if

	// not alarm mode
	else
	{
		alarm_ack_timer = 0; 					//clear
		flag_alarm_ack = 0;						//claer
		buzzer_off();																							// BUZZER OFF
 		lamp_safe();																							// red lamp off, green lamp on
	}//else

}//void

/* Function to calculate absolute value of two numbers */
float abs_diff(float a, float b) {
  float diff = a - b;
  if (diff < 0) {
    diff = -diff;
  }
  return diff;
}//float

/* Function to calculate exponential moving average */
void process_dose_ema()
{

	// get timer count value for moving average
	//low Range
	gmCountLowFiltered = gmCountLowFiltered_prev * (1.000F - alpha_lo) + (float)gmCountLow * alpha_lo;
	gmCountLowFiltered_prev = gmCountLowFiltered;

	// low range reference ema
	gmCountLow_ref = gmCountLow_ref_prev * (1.000F - alpha_lo_ref) + (float)gmCountLow * alpha_lo_ref;
	gmCountLow_ref_prev = gmCountLow_ref;

	// convert average time to ema alpha parameter.
	// Subtract a noise deadband from the ref-vs-filtered divergence so background Poisson
	// jitter does not keep alpha_lo elevated; clamp negative to 0 before sqrt.
	float div_lo = abs_diff(gmCountLow_ref, gmCountLowFiltered) - EMA_DEADBAND_LOW;
	if(div_lo < 0.0F)
		div_lo = 0.0F;
	alpha_lo = 0.150F * sqrt(div_lo);

	// set limit, alpha_lo is 0 < alpha_lo < 1
	if(alpha_lo < ALPHA_LO_MIN)		//minimum alpha = longest averaging when stable (background)
		alpha_lo = ALPHA_LO_MIN;
	if(alpha_lo > 0.45F)		//maximum average time 1s limit, 10sec
		alpha_lo = 0.45F;

	// calculate alpha_ref, alpha_ref is twice the value of alpha_lo
	alpha_lo_ref = alpha_lo * 2.00F;

	// set limit, alpha_lo is 0 < alpha_lo < 1
	if(alpha_lo_ref < 0.02F)		//minimum average time 60s limit, 60sec, minimum 0.017
		alpha_lo_ref = 0.02F;
	if(alpha_lo_ref > 0.90F)		//maximum average time 1s limit, 10sec
		alpha_lo_ref = 0.90F;


	//high Range///////////////////////////////////////////////////////////////////////////////////////////////////
	gmCountHighFiltered = gmCountHighFiltered_prev * (1.000F - alpha_hi) + (float)gmCountHigh * alpha_hi;
	gmCountHighFiltered_prev = gmCountHighFiltered;

	// low range reference ema
	gmCountHigh_ref = gmCountHigh_ref_prev * (1.000F - alpha_hi_ref) + (float)gmCountHigh * alpha_hi_ref;
	gmCountHigh_ref_prev = gmCountHigh_ref;

	// convert average time to ema alpha parameter
//	alpha_hi = 0.15F * sqrt(abs_diff(gmCountHigh_ref, gmCountHighFiltered));
	alpha_hi = 0.10F * sqrt(abs_diff(gmCountHigh_ref, gmCountHighFiltered));

	// alpha_hi is 0 < alpha_hi < 1
	if(alpha_hi < 0.01F)			//minimum average time 100s limit
		alpha_hi = 0.01F;
	if(alpha_hi > 0.10F)			//maximum average time 10s limit
		alpha_hi = 0.10F;
//	if(alpha_hi > 0.45F)			//maximum average time 10s limit
//		alpha_hi = 0.45F;

	// calculate alpha_ref, alpha_ref is twice the value of alpha_lo
	alpha_hi_ref = alpha_hi * 2.00F;

	// set limit, alpha_hi is 0 < alpha_hi < 1
	if(alpha_hi_ref < 0.02F)		//minimum average time 100s limit
		alpha_hi_ref = 0.02F;
	if(alpha_hi_ref > 0.20F)		//maximum average time 10s limit
		alpha_hi_ref = 0.20F;
//	if(alpha_hi_ref > 0.90F)		//maximum average time 10s limit
//		alpha_hi_ref = 0.90F;

	// apply n/(1-nt) dead-time correction, with denominator clamp to prevent blow-up
	float tau_denom = 1.0F - (gmCountLowFiltered * TAULOW);
	if (tau_denom < 0.1F) tau_denom = 0.1F;		// clamp: avoid divide-by-~0 / runaway dose
	gm_tau_low = gmCountLowFiltered / tau_denom;

	/* calculate dose rate
	 * Conversion Factor LND7128	= 1.43
	 * Conversion Factor LND7132	= 0.556
	 * Conversion Factor LND71631 = 53.625
	*/
	gmDoseLow 	= gm_tau_low  * conv_factor_low;
	gmDoseHigh 	= gmCountHighFiltered * conv_factor_high;

	//if below 0.05, display minimum dose & check no count
	check_no_count();

}//void

void process_range_check()
{

  switch (rangeStatus)
  {
  case RANGE_LOW:
//    if(gmDoseLow > RANGE_VAL_HIGH)
    if(gmDoseHigh > RANGE_VAL_HIGH)		// Range change bug fix 24/04/12
    {
    	rangeStatus = RANGE_HIGH;		// change Range LOW -> HIGH
    }//if
    else
    {
    	rangeStatus = RANGE_LOW;		// no change
    }//else
    break;

  case RANGE_HIGH:
    if(gmDoseHigh < RANGE_VAL_LOW)
    {
    	rangeStatus = RANGE_LOW;		// change Range HIGH -> LOW
    }//if
    else
    {
    	rangeStatus = RANGE_HIGH;		// no change
    }//else
    break;

  }//switch

}//void

void data_monitor_usb()
{
	printf("ref: %05d, ", RANGE_VAL_CENTER);

	if(rangeStatus == 0)
		printf("Range: %s, ", "L");
	else
		printf("Range: %s, ", "H");

	printf("CntL: %.2f, CntH: %.2f ", gmCountLowFiltered, gmCountHighFiltered);

	printf("tau: %.3f ", gm_tau_low);

	printf("Lo: %.2f, Hi: %.2f\r\n", gmDoseLow, gmDoseHigh);

//	printf("al: %.2f, b: %.2f, diff: %.2f, alpha: %.2f\r\n", gmCountLow_ref, gmCountLowFiltered, gmCountLow_ref-gmCountLowFiltered, alpha_lo);
//	printf("ah: %.2f, b: %.2f, diff: %.2f, alpha: %.2f\r\n", gmCountHigh_ref, gmCountHighFiltered, gmCountHigh_ref-gmCountHighFiltered, alpha_hi);
}//void

void test_gmDoseLow_display()
{
		gmDoseLow = gmDoseLow + 0.05F;

		if(gmDoseLow > 1)
			gmDoseLow = gmDoseLow + 0.5F;

		if(gmDoseLow > 10)
			gmDoseLow = gmDoseLow + 5.0F;

		if(gmDoseLow > 100)
			gmDoseLow = gmDoseLow + 50.0F;

		if(gmDoseLow > 1000)
			gmDoseLow = gmDoseLow + 500.0F;

		if(gmDoseLow > 10000)
			gmDoseLow = gmDoseLow + 5000.0F;
}//void

void test_gmDoseHigh_display()
{

		gmDoseHigh = gmDoseHigh + 0.05F;

		if(gmDoseHigh > 1)
			gmDoseHigh = gmDoseHigh + 0.5F;

		if(gmDoseHigh > 10)
			gmDoseHigh = gmDoseHigh + 5.0F;

		if(gmDoseHigh > 100)
			gmDoseHigh = gmDoseHigh + 50.0F;

		if(gmDoseHigh > 1000)
			gmDoseHigh = gmDoseHigh + 500.0F;

		if(gmDoseHigh > 10000)
			gmDoseHigh = gmDoseHigh + 5000.0F;

}//void

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* NOTE: MX_IWDG_Init() must NOT run here. The watchdog has to start only AFTER the
     long blocking startup (init_display switch-waits, power_on_selftest ~3.5s, EEPROM
     loads); otherwise it resets during POST -> boot loop. It is started later, just
     before the main loop. If CubeMX regenerates this block and re-adds MX_IWDG_Init()
     here, delete it again. */
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //Start Timer4 with interrupt
  if(HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
  	//*Starting Error
  	Error_Handler();
  }

  //Start Timer1
  if(HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
  	//*Starting Error
  	Error_Handler();
  }

  //Start Timer8
  if(HAL_TIM_Base_Start(&htim8) != HAL_OK)
  {
  	//*Starting Error
  	Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init_system();
  init_display();

  power_on_selftest();		// startup check : display, LEDs, buzzer

  HAL_Delay(1000);

  //load alarm data from EEPROM
  load_alarm_data();
  load_cf_data();

  /* Start the independent watchdog AFTER all blocking startup (POST, EEPROM load,
     and the switch-hold waits in init_display). ~4 s timeout; kicked each loop. */
  __HAL_DBGMCU_FREEZE_IWDG();		// keep IWDG halted while debugging (no reset on breakpoints)
  MX_IWDG_Init();

  while (1)
  {
  	process_switch();

//  	flag_movingAverage = 1;	//set flag
//  	if(flag_movingAverage == 1)
  	if(tick_1_0sec == 1)				//every 1s
  	{
  		tick_1_0sec = 0;		//clear

    	process_alarm();
  		process_range_check();
  		process_dose_ema();
  		data_monitor_usb();			// data transfer to a SmartPhone
  	}//if

  	if(tick_0_2sec == 1)						//every 0.2s
  	{
  		tick_0_2sec = 0; 					//clear

  		// Kick the watchdog only here, i.e. only when the 1ms tick source (TIM4) and the
  		// display path are actually progressing. If ticks stop or display hangs, this
  		// block stops running -> IWDG (~4s) resets the unit. (200ms period << 4s timeout)
  		HAL_IWDG_Refresh(&hiwdg);

  		if(flag_display_test == 1)			// select display test
  		{
  			display_monitor();
  		}//if

  		else if(flag_display_cf == 1)		// select display conversion factor
  		{
  			display_cf();
  		}//else if

  		else
  		{
  			display_dose();								// normal display
  		}//else
  	}//if

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}//while
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 500-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 49;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 269;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 135;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|ETH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LAMP_RED_Pin|BT_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EE_WP_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin ETH_CS_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|ETH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_RST_Pin */
  GPIO_InitStruct.Pin = ETH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ETH_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_INT_Pin */
  GPIO_InitStruct.Pin = ETH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ETH_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_DOWN_Pin SW_UP_Pin SW_SHIFT_Pin SW_SET_Pin */
  GPIO_InitStruct.Pin = SW_DOWN_Pin|SW_UP_Pin|SW_SHIFT_Pin|SW_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LAMP_RED_Pin BT_ON_Pin */
  GPIO_InitStruct.Pin = LAMP_RED_Pin|BT_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : EE_WP_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = EE_WP_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//time tick every 0.1s
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		gTimerCnt++;

		if(gTimerCnt % 100 == 0)		//every 0.1s
		{
			tick_0_1sec = 1;
		}//if

		if(gTimerCnt % 200 == 0)		//every 0.2s
		{
			tick_0_2sec = 1;
		}//if

		if(gTimerCnt % 500 == 0)		//every 0.5s
		{
			tick_0_5sec = 1;
		}//if

		if(gTimerCnt == 1000)				//every 1s
		{
			gTimerCnt = 0;	//clear parameter

			tick_1_0sec = 1;

			// GM channels swapped (wiring + labels): LOW tube now on PA0/TIM8, HIGH tube on PE7/TIM1.
			// So read the LOW count from TIM8 and the HIGH count from TIM1 (swapped vs original).
			gmCountLow = __HAL_TIM_GET_COUNTER(&htim8);		//get gm low count value  (PA0 / TIM8)
			__HAL_TIM_SetCounter(&htim8, 0);							//init counter

			gmCountHigh = __HAL_TIM_GET_COUNTER(&htim1);	//get gm high count value (PE7 / TIM1)
			__HAL_TIM_SetCounter(&htim1, 0);							//init counter

			current_avg_time++;					//increase average time every 1sec
		}//if
	}//if
}//void

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case SW_SET_Pin :
		flag_SW_SET = 1;
		break;
	case SW_SHIFT_Pin :
		flag_SW_SHIFT = 1;
		break;
	case SW_UP_Pin :
		flag_SW_UP = 1;
		break;
	case SW_DOWN_Pin :
		flag_SW_DOWN = 1;
		break;
	// SW_ACK_Pin removed : not assigned in current .ioc
	default:
		;
	}//switch
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    /* Indicate a fatal fault: blink the red lamp so it is not a silent hang.
       IRQs are disabled here (no SysTick), so use a busy-wait delay. */
    HAL_GPIO_TogglePin(LAMP_RED_GPIO_Port, LAMP_RED_Pin);
    for (volatile uint32_t d = 0; d < 3000000; d++) { __NOP(); }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
