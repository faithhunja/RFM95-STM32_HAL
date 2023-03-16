/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define MASTER //Sender
#define SLAVE //Receiver(s)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int test = 0;
uint8_t* test_var = (uint8_t *)"test";
uint8_t* test1_var = (uint8_t *)"four";
uint8_t buffer[32];
uint8_t packet_avail = 0;
char instruct[50];
char version[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
  lora_sx1276 lora;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LoRa module
  uint8_t init = lora_init(&lora, &hspi1, GPIOA, GPIO_PIN_4, LORA_BASE_FREQUENCY_EU);
  if (init != LORA_OK) {
	// Initialization failed
    test = 404;
  } else
	  test = 505;

//  uint8_t* ver = (uint8_t *)lora_version(&lora);

  char version = lora_version(&lora);
  uint8_t ver = lora_version(&lora);
  if (ver != LORA_COMPATIBLE_VERSION) {
    sprintf(instruct, "Got wrong radio version 0x%x, expected 0x12", ver);
    sprintf(version, "0x%x", ver);
    return LORA_ERROR;
  }
//  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//  {
//      if (GPIO_Pin == DIO0_Pin)
//      {
//  		#ifdef SLAVE
//      	lora_enable_interrupt_rx_done(&lora);
//  		#endif
//
//  		#ifdef MASTER
//      	lora_enable_interrupt_tx_done(&lora);
//  		#endif
//      }
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	#ifdef SLAVE
	lora_mode_standby(&lora);
	test = 606;
    // Send packets in blocking mode
	lora_send_packet(&lora, test_var, 4);
	HAL_Delay(5);
	lora_send_packet(&lora, test1_var, 4);
	HAL_Delay(5);

	uint8_t packet_avail = lora_is_transmitting(&lora);
		if (packet_avail == 0)
			sprintf(instruct, "No packet transmission");
		else
			sprintf(instruct, "Transmitting packet");

	#endif

	#ifdef MASTER
	// Standby mode
	lora_mode_standby(&lora);
	uint8_t packet_avail = lora_is_packet_available(&lora);
	if (packet_avail == 0)
		sprintf(instruct, "No packet available");
	else
		sprintf(instruct, "Packet ready");

	// Put LoRa modem into continuous receive mode
	lora_mode_receive_continuous(&lora);
	// Wait for packet up to 10sec
	uint8_t res;
	uint8_t len = lora_receive_packet_blocking(&lora, buffer, sizeof(buffer), 10000, &res);
	if (res != LORA_OK) {
	  // Receive failed
		test = 505;
	}
	test = 707;
	buffer[len] = 0;  // null terminate string to print it
//	printf("'%s'\n", buffer);
	#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  lora_mode_sleep(&lora);

//  #ifdef SLAVE
//  lora_clear_interrupt_tx_done(&lora);
//  #endif
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
