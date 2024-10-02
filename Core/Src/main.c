/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "stdbool.h"
#include "Black_Pill_Blinker_.h"

#define APP_START_ADDRESS 0x0800C000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t FLASH_START_ADDRESS = 0x000100;

uint32_t bytes_to_programm = 0;

uint8_t Write_Enable_cmd = 0x06;
uint8_t RX_Data_1 = 0;
uint8_t RX_Data_256 [256] = {0};
uint8_t TX_Data_256 [256] = {0};
uint8_t Sector_Erase_cmd = 0x20;
uint8_t Dummy_Byte = 0x00;
uint8_t Read_Status_Register_cmd = 0x05;
uint8_t Page_Program = 0x02;
uint8_t Read_Data_cmd = 0x03;
uint8_t TX_Sector_Erase [4];
uint8_t TX_Page_Program [4];
uint8_t TX_Read_Data [4];

uint32_t Bytes_written_counter = 0;
uint32_t Pages_printed_counter = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
extern const unsigned char my_arr[8712];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void WriteEnable (void);
void Sector_Erase (uint32_t sector);
void Write_Page (uint32_t addr);
//void Read_Page (uint32_t addr);
void Read_Data(uint32_t addr);
void Write_Firmware(const uint8_t * firmware, uint32_t flash_start_addr);
int Check_Ready_to_Write (void);
int Check_BUSY_bit (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
}
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
  bytes_to_programm = sizeof(my_arr);
  uint8_t sectors_to_erase = 0;

  if(bytes_to_programm % 0x1000) {							// делим на минимальный блок для очистки, т.е. сектор (4КБ)
	  sectors_to_erase = (bytes_to_programm / 0x1000) + 1;
  } else {
	  sectors_to_erase = bytes_to_programm / 0x1000;
  }
  printf("sectors_to_erase = %d, remainder of the division = %" PRId32 "  \n", sectors_to_erase, (bytes_to_programm % 0x1000));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("LED begin\n");														// Начинаем шить флеш, включаем диод
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  for(int i = 0; i < sectors_to_erase; i++) {									// Стираем нужные сектора (4КБ), проверяя бизи бит
	WriteEnable();																// Включаем запись флеш и проверяем
	if(Check_Ready_to_Write()) printf("WEN OK \n");
	else printf("WEN NOK \n");

	printf("Erase sector %d \n", i);
    Sector_Erase(i);
    while(Check_BUSY_bit()){
	  printf("BUSY \n");
	  HAL_Delay(50);
    }
  }

  WriteEnable();																// Включаем запись флеш и проверяем
  if(Check_Ready_to_Write()) printf("WEN OK \n");
  else printf("WEN NOK \n");

  Write_Firmware(my_arr, 0);													// Шьем прошивку во флеш

  for(int i = 0; i < 2; i++){
  Pages_printed_counter = 0;
  uint32_t errors_counter = 0;
  for (uint32_t i = 0; i < bytes_to_programm; i += 0x100) {						// Читаем и выводим постранично
	Read_Data(i);

	printf("Page: %" PRId32 " \n", Pages_printed_counter);						// Выводим в 16м формате по странице
	for(int x = 0; x < 16; x++){
  	  for(int i = 0; i < 16; i++){				 								// Квадратом 16 на 16
  		printf("0x%02X ", RX_Data_256[i + x*16]);
  	  }
  	  printf("\n");
  	  //HAL_Delay(100);
  	}
  	printf("\n");

  	if(i + 0x100 < bytes_to_programm) {											// Если выводим полную страницу - сохраняем в буфер 256 байт
  	  for(uint32_t x = 0; x < 256; x++){
  	    TX_Data_256[x] = my_arr[i + x]; 										// TX_Data_256 используем как проверочный буфер, один хрен он больше не нужен
  	  }
  	} else {																	// Если выводим не полную (последнюю) страницу
  		for(uint32_t x = 0; x < bytes_to_programm - i; x++){					// TX_Data_256 используем как проверочный буфер, один хрен он больше не нужен
  		  TX_Data_256[x] = my_arr[i + x];
  		}
  		for(uint32_t x = (bytes_to_programm - i); x < 256; x++){				// Остальное забиваем единицами
  		  TX_Data_256[x] = 0xFF;
  		}
  	}

  	if(memcmp(TX_Data_256, RX_Data_256, (sizeof(uint8_t)*256))) {				// Проверка на совпадение
  		printf("NOT equal \n");
  		errors_counter++;

  		for(int x = 0; x < 16; x++){											// Если не совпало, выводим что не совпало
  	  		for(int i = 0; i < 16; i++){
  	  			printf("0x%02X ", TX_Data_256[i + x*16]);						// Выводим в 16м формате
  	  		}
  	  		printf("\n");
  	  		//HAL_Delay(100);
  	  	}
  	  	printf("\n");
  	} else {
  		printf("Page is OK \n");
  	}
  	printf("\n");

  	Pages_printed_counter++;
  }

  printf("Errors: %" PRId32 " \n", errors_counter);
  printf("\n");
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  //HAL_Delay(500);
  printf("LED end\n");

//  for(uint8_t i = 0; i < 16; i++) {
//	  printf("0x%X ", my_arr[i]);
//  }
//  printf("\n");



  uint32_t appJumpAddress;
  appJumpAddress = * ((volatile uint32_t*)(APP_START_ADDRESS + 4));

  HAL_SPI_DeInit(&hspi1);
  HAL_RCC_DeInit();
  HAL_DeInit();

  void(*GoToApp)(void);
  GoToApp = (void (*)(void)) appJumpAddress;

  __disable_irq();
  __set_MSP(*((volatile uint32_t*)APP_START_ADDRESS));
  GoToApp();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//printf("tick\n");
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SPI_Pin */
  GPIO_InitStruct.Pin = CS_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_SPI_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void WriteEnable (void){
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// write enable
	HAL_SPI_Transmit(&hspi1, &Write_Enable_cmd, sizeof(Write_Enable_cmd), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);
}

int Check_Ready_to_Write (void){
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// read status register
	HAL_SPI_Transmit(&hspi1, &Read_Status_Register_cmd, sizeof(Read_Status_Register_cmd), 1000);
	HAL_SPI_Receive(&hspi1, &RX_Data_1, sizeof(RX_Data_1), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);

	return ((RX_Data_1 >> 1) & 1);
}

void Sector_Erase (uint32_t sector){
	uint32_t sector_addr = sector * 0x1000;
	TX_Sector_Erase[0] = Sector_Erase_cmd;
	TX_Sector_Erase[1] = (uint8_t)(sector_addr >> 16) & 0xFF;
	TX_Sector_Erase[2] = (uint8_t)(sector_addr >> 8) & 0xFF;
	TX_Sector_Erase[3] = (uint8_t)(sector_addr & 0xFF);

	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// sector erase
	HAL_SPI_Transmit(&hspi1, TX_Sector_Erase, sizeof(TX_Sector_Erase), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);
}

int Check_BUSY_bit (void){
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// read status register
	HAL_SPI_Transmit(&hspi1, &Read_Status_Register_cmd, sizeof(Read_Status_Register_cmd), 1000);
	HAL_SPI_Receive(&hspi1, &RX_Data_1, sizeof(RX_Data_1), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);

	return (RX_Data_1 & 1);
}

void Write_Page(uint32_t addr){
	TX_Page_Program [0] = Page_Program;
	TX_Page_Program [1] = (uint8_t)(addr >> 16) & 0xFF;
	TX_Page_Program [2] = (uint8_t)(addr >> 8) & 0xFF;
	TX_Page_Program [3] = (uint8_t)(addr & 0xFF);

	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// write first page
	HAL_SPI_Transmit(&hspi1, TX_Page_Program, sizeof(TX_Page_Program), 1000);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&my_arr[0], 256, 2000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);
}

void Read_Data(uint32_t addr){
	TX_Read_Data [0] = Read_Data_cmd;
	TX_Read_Data [1] = (uint8_t)(addr >> 16) & 0xFF;
	TX_Read_Data [2] = (uint8_t)(addr >> 8) & 0xFF;
	TX_Read_Data [3] = (uint8_t)(addr & 0xFF);

	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// read first page
	HAL_SPI_Transmit(&hspi1, TX_Read_Data, sizeof(TX_Read_Data), 1000);
	HAL_SPI_Receive(&hspi1, RX_Data_256, sizeof(RX_Data_256), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);
}

void Write_Firmware(const uint8_t * firmware, uint32_t flash_start_addr) {
	uint32_t bytes_is_programmed = 0;
	uint8_t counter = 0;

	while(bytes_to_programm - bytes_is_programmed >= 0x100) {
	  TX_Page_Program [0] = Page_Program;
	  TX_Page_Program [1] = (uint8_t)(flash_start_addr >> 16) & 0xFF;
	  TX_Page_Program [2] = (uint8_t)(flash_start_addr >> 8) & 0xFF;
	  TX_Page_Program [3] = (uint8_t)(flash_start_addr & 0xFF);

	  counter = 0;
	  for(int i = bytes_is_programmed; i < (bytes_is_programmed + 256); i++) {
		TX_Data_256[counter] = firmware[i];
		counter++;
	  }

	  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, TX_Page_Program, sizeof(TX_Page_Program), 1000);
	  //HAL_SPI_Transmit(&hspi1, firmware[bytes_is_programmed], 256, 2000);		// шьем страницу флеш с нужным смещением по массиву
	  HAL_SPI_Transmit(&hspi1, TX_Data_256, 256, 2000);
	  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	  //HAL_Delay(50);

	  while(Check_BUSY_bit()){
		//printf("BUSY \n");
	  }

	  bytes_is_programmed += 0x100;
	  flash_start_addr += 0x100;

	  if(bytes_to_programm - bytes_is_programmed >= 0x100) {
		  WriteEnable();																	// Включаем запись флеш и проверяем
		  //if(Check_Ready_to_Write()) printf("WEN OK \n");
		  //else printf("WEN NOK \n");
	  }
	}

	if(bytes_to_programm - bytes_is_programmed > 0) {
	  WriteEnable();																	// Включаем запись флеш и проверяем
	  if(Check_Ready_to_Write()) printf("WEN OK \n");
	  else printf("WEN NOK \n");

	  TX_Page_Program [0] = Page_Program;
	  TX_Page_Program [1] = (uint8_t)(flash_start_addr >> 16) & 0xFF;
	  TX_Page_Program [2] = (uint8_t)(flash_start_addr >> 8) & 0xFF;
	  TX_Page_Program [3] = (uint8_t)(flash_start_addr & 0xFF);

	  counter = 0;
	  for(uint32_t i = bytes_is_programmed; i < bytes_to_programm; i++) {				// Дописываем оставшиеся байты
		TX_Data_256[counter] = firmware[i];
		counter++;
	  }

	  for(int i = 0; i < (256 - (bytes_to_programm % 256)); i++) {						// Остальные байты страницы забиваем единицами
	  	TX_Data_256[counter] = 0xFF;
	  	counter++;
	  }

	  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, TX_Page_Program, sizeof(TX_Page_Program), 1000);
	  //HAL_SPI_Transmit(&hspi1, (uint8_t *)&firmware[bytes_is_programmed], (bytes_to_programm - bytes_is_programmed), 2000);		// шьем страницу флеш с нужным смещением по массиву
	  HAL_SPI_Transmit(&hspi1, TX_Data_256, 256, 2000);
	  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	  //HAL_Delay(50);

	  while(Check_BUSY_bit()){
		printf("BUSY \n");
	  }
}
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
  }
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
