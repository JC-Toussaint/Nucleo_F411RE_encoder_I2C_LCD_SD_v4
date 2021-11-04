/* USER CODE BEGIN Header */

//https://deepbluembedded.com/stm32-timer-encoder-mode-stm32-rotary-encoder-interfacing/

// ENCODER
// PA0     ------> TIM2_CH1
// PA1     ------> TIM2_CH2
// In stm32f4xx_hal_msp.c change GPIO_NOPULL to GPIO_PULLUP
// GPIO_InitStruct.Pull = GPIO_PULLUP;

// LCD pinout
// PB9    ------> I2C1_SDA
// PB8    ------> I2C1_SDL
// #define SLAVE_ADDRESS_LCD (0x20<<1)  in i2c_HD44780.c file

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <i2c_HD44780.h>
#include "emdlist.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	SHIELD_NOT_DETECTED = 0,
	SHIELD_DETECTED
}ShieldStatus;

/* Private define ------------------------------------------------------------*/
#define SD_CARD_NOT_FORMATTED                    0
#define SD_CARD_FILE_NOT_SUPPORTED               1
#define SD_CARD_OPEN_FAIL                        2
#define FATFS_NOT_MOUNTED                        3
#define STRING_SZ                              256
#define ITEMS_MAXSZ                            256

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool SELECT_BUTTON_state = true;

uint8_t BlinkSpeed = 0, str[20];
__IO uint8_t JoystickValue = 0;
char* pDirectoryFiles[MAX_BMP_FILES];
DIR dir;
FILINFO fno;
FIL MyFile;     /* File object */
FATFS SD_FatFs;  /* File system object for SD card logical drive */
char SD_Path[4]; /* SD card logical drive path */
DIR  items[ITEMS_MAXSZ];   /* array of dir struct */
UINT items_sz=0;

DoubleLinkedList dlist, dlist_clean;
char buffer[STRING_SZ];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

FRESULT scan_files (char* path, DoubleLinkedList* dlist);
FRESULT read_filename(char* path, DIR dir, char* fname);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// WARNING UART2 is connected to st-link
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
	return len;
}

/*
int _write(int file, char *ptr, int len)
{
	for (uint8_t i=0; i<len; i++){
		LL_USART_TransmitData8(USART2, ptr[i]);
		while ( !LL_USART_IsActiveFlag_TXE(USART2) )
		{
		}
	}
	return len;
}

int UART_Transmit(USART_TypeDef *UART, uint8_t *ptr, int len){
	for (uint8_t i=0; i<len; i++){
		LL_USART_TransmitData8(UART, ptr[i]);
		while ( !LL_USART_IsActiveFlag_TXE(UART) )
		{
		}
	}
	return 0;
}
 */

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
	HAL_Delay(100);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_SPI1_Init();
	//MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	HAL_Delay(2);

	printf("Start\n");

	lcd_init ();

	lcd_clear_display();
	lcd_home();

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	/*##-1- Link the SD disk I/O driver ########################################*/
	if(FATFS_LinkDriver(&SD_Driver, SD_Path) != FR_OK) Error_Handler();


	/*##-2- Register the file system object to the FatFs module ##############*/
	if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) != FR_OK) Error_Handler();

	/*##-3- Create a FAT file system (format) on the logical drive #########*/
	char path[2]="/";
	FRESULT res = scan_files(path, &dlist);
	if (res != FR_OK) Error_Handler();

	printf("==== list of files =======\n");

	{ // list of files
		DoubleLinkedListIterator iterator = emdlist_iterator(&dlist);
		DoubleLinkedListElement* candidate = NULL;

		// emdlist_iterator_next(&iterator) pointer to the first element
		while((candidate = emdlist_iterator_next(&iterator)) != NULL) {
			DIR *dir = candidate->value;
			snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
			printf("%s\n", buffer);
			FRESULT res = read_filename(path, *dir, buffer);
			if (res != FR_OK) continue;
			printf("%s\n", buffer);
		}
	}

	printf("==============================================\n");

	/*##-7- Open the text file object with read access ###############*/
	{ // content of files
		DoubleLinkedListIterator iterator = emdlist_iterator(&dlist);
		DoubleLinkedListElement* candidate = NULL;
		while((candidate = emdlist_iterator_next(&iterator)) != NULL) {
			DIR *dir = candidate->value;
			snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
			printf("%s\n", buffer);
			FRESULT res = read_filename(path, *dir, buffer);
			if (res != FR_OK) continue;

			emdlist_pushfront(&dlist_clean, (void*) dir); // AJOUT 02/11/2021

			buffer[strcspn(buffer, "\r\n")] = 0; // works for LF, CR, CRLF, LFCR, ...
			printf("filename : %s\n", buffer);
			if(f_open(&MyFile, buffer, FA_READ) != FR_OK)
			{
				/* file Open for read Error */
				//Error_Handler();
				printf("can't open %s\n ... next one", buffer);
				continue;
			}

			//     DO NOT FORGET TO #define	_USE_STRFUNC	1  in ffconf.h
			while (f_gets(buffer, STRING_SZ, &MyFile) ){
				/* writing content to stdout */
				printf("%s", buffer);
			}
			f_close(&MyFile);
		}
	}

	printf("====== clean files list ========\n");

	{ // list of files
		DoubleLinkedListIterator iterator = emdlist_iterator(&dlist_clean);
		DoubleLinkedListElement* candidate = NULL;
		// emdlist_iterator_next(&iterator) pointer to the first element

		while((candidate = emdlist_iterator_next(&iterator)) != NULL) {
			DIR *dir = candidate->value;
			printf("current  %x  next %x prev %x\n", candidate, candidate->next, candidate->prev);
			printf("# iterator.next %x iterator.prev %x\n", iterator.next, iterator.prev);
			snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
			printf("%s\n", buffer);
			FRESULT res = read_filename(path, *dir, buffer);
			if (res != FR_OK) continue;
			printf("filename%s\n", buffer);
		}
	}
	/*##-11- Unlink the SD disk I/O driver ####################################*/
	//FATFS_UnLinkDriver(SD_Path);

	printf("SCROLLING\n");

	DoubleLinkedListIterator iterator = emdlist_iterator(&dlist_clean);
	//DoubleLinkedListElement* pos = emdlist_iterator_next(&iterator);
	int32_t cnt0 = (int32_t) __HAL_TIM_GET_COUNTER(&htim2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//printf("%lu \n", __HAL_TIM_GET_COUNTER(&htim2));

		char str[21];
		int32_t cnt = (int32_t) __HAL_TIM_GET_COUNTER(&htim2);
		int offset=cnt-cnt0;
		cnt0=cnt;
		snprintf(str, 21, "%10d%10d", cnt, offset);
		lcd_locate(1, 1);
		lcd_print_string (str);

		if (offset>=0){
			printf("offset %d\n", offset);
			HAL_Delay(2000);
			for (int it=0; it<offset; it++){
				printf("next\n");
				emdlist_iterator_next(&iterator);
			}
			// emdlist_iterator_next(&iterator) pointer to the first element
			DoubleLinkedListIterator local_iterator = iterator;
			DoubleLinkedListElement* candidate = NULL;
			while((candidate = emdlist_iterator_next(&local_iterator)) != NULL) {
				DIR *dir = candidate->value;
				snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
				printf("%s\n", buffer);
				FRESULT res = read_filename(path, *dir, buffer);
				if (res != FR_OK) continue;
				printf("%s\n", buffer);
			}

//			DoubleLinkedListIterator local_iterator = iterator;
//			for (int nrow=0; nrow<4; nrow++){
//				DoubleLinkedListElement* candidate = emdlist_iterator_next(&local_iterator);
//				if (candidate == NULL) break;
//				DIR *dir = candidate->value;
//				snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
//				printf("%s\n", buffer);
//				FRESULT res = read_filename(path, *dir, buffer);
//				if (res != FR_OK) continue;
//
//				buffer[strcspn(buffer, "\r\n")] = 0; // works for LF, CR, CRLF, LFCR, ...
//				printf("filename : %s\n", buffer);
//			}
		}
		if (offset<0){
			printf("offset %d\n", offset);
			HAL_Delay(2000);
			for (int it=0; it>offset; it--){
				printf("prev\n");
				emdlist_iterator_prev(&iterator);
			}
			// emdlist_iterator_next(&iterator) pointer to the first element
			DoubleLinkedListIterator local_iterator = iterator;
			DoubleLinkedListElement* candidate = NULL;
			while((candidate = emdlist_iterator_prev(&local_iterator)) != NULL) {
				DIR *dir = candidate->value;
				snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
				printf("%s\n", buffer);
				FRESULT res = read_filename(path, *dir, buffer);
				if (res != FR_OK) continue;
				printf("%s\n", buffer);
			}

//			DoubleLinkedListIterator local_iterator = iterator;
//			for (int nrow=0; nrow<4; nrow++){
//				DoubleLinkedListElement* candidate = emdlist_iterator_next(&local_iterator);
//				if (candidate == NULL) break;
//				DIR *dir = candidate->value;
//				snprintf(buffer, sizeof(buffer), "dir %p", dir->dir);
//				printf("%s\n", buffer);
//				FRESULT res = read_filename(path, *dir, buffer);
//				if (res != FR_OK) continue;
//
//				buffer[strcspn(buffer, "\r\n")] = 0; // works for LF, CR, CRLF, LFCR, ...
//				printf("filename : %s\n", buffer);
//			}
		}



		//HAL_Delay(500);
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
	htim1.Init.Prescaler = 63;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 62499;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
	huart2.Init.BaudRate = 115200;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SELECT_BUTTON_Pin */
	GPIO_InitStruct.Pin = SELECT_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SELECT_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_CS_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if(GPIO_Pin == GPIO_PIN_0) {
//    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
//  } else {
//      __NOP();
//  }
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SELECT_BUTTON_Pin && SELECT_BUTTON_state == true){
		HAL_TIM_Base_Start_IT(&htim1);
		printf("GPIO interrupt TIM1 started\n");
		SELECT_BUTTON_state = false;
	}
	else{
		__NOP();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	 */
	if (htim->Instance==TIM1){
		if(HAL_GPIO_ReadPin(SELECT_BUTTON_GPIO_Port, SELECT_BUTTON_Pin) == GPIO_PIN_RESET){
			SELECT_BUTTON_state = true;
			printf("SELECT_BUTTON pressed %lu\n", __HAL_TIM_GET_COUNTER(&htim2));
			HAL_TIM_Base_Stop_IT(&htim1);
		}
	}
}

FRESULT scan_files (char* path, DoubleLinkedList* dlist)  /* Start node to be scanned (***also used as work area***) */
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;
	char buffer[256];

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		DIR *item = (DIR*) malloc(sizeof(DIR));
		if (!item) Error_Handler();
		*item=dir;
		//emdlist_pushfront(dlist, (void*) item);
		for (;;) {
			res = f_readdir(&dir, &fno);                   /* Read a directory item */

			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			//			if (fno.fattrib & AM_DIR) {                    /* It is a directory */
			//				i = strlen(path);
			//				sprintf(&path[i], "/%s", fno.fname);
			//				res = scan_files(path, items, items_sz);     /* Enter the directory */
			//				if (res != FR_OK) break;
			//				path[i] = 0;
			//			} else
			{                                       /* It is a file. */
				snprintf(buffer, sizeof(buffer), "%s/%s\n", path, fno.fname);
				printf("%s\n", buffer);
				DIR *item = (DIR*) malloc(sizeof(DIR));
				if (!item) Error_Handler();
				*item=dir;
				emdlist_pushfront(dlist, (void*) item);

				snprintf(buffer, sizeof(buffer), "DIR.dir pointer %p\n", dir.dir);
				printf("%s\n", buffer);
			}
		}
		f_closedir(&dir);
	}

	return res;
}

FRESULT read_filename(char* path, DIR target_dir, char* fname)        /* Start node to be scanned (***also used as work area***) */
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		res = f_readdir(&target_dir, &fno);                   /* Read a directory item */

		if (res != FR_OK || fno.fname[0] == 0) {
			fname[0]=0;
			f_closedir(&dir);
			return FR_INT_ERR;  /* Break on error or end of dir */
		}
		if (fno.fattrib & AM_DIR) {
			fname[0]=0;
			f_closedir(&dir);
			return FR_INT_ERR;
		}

		snprintf(fname, STRING_SZ, "%s/%s", path, fno.fname);
	}
	f_closedir(&dir);

	return res;
}

void print(DoubleLinkedListElement* candidate){
	DIR *e=(DIR*) candidate->value;
	printf("%p prev %p next %p", candidate, candidate->prev, candidate->next);
	if (e) printf(" element %p --> %p", e, e->dir);
	printf("\n");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
