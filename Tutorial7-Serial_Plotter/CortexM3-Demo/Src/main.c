/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @author         : Chan Tai Wing Vincent
	* @content        : M3 Demo - Serial Plotter
	* @version        : Tutorial 7
  ******************************************************************************
  * @attention
  * This program is written for STM32F103RB Nucleo Board. This program is to 
  * demonstrate how to control the built in LED (ON/OFF) by the USART message
  * which controlled by RX interrupt. Also, send USART message to another deivce.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h" //include for bool type
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool LED_ON = false;
unsigned char USART_RX_BUFFER[3];
unsigned char SYSTEM_START[28] = "WELCEOM TO STM32F103RBT6 !\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		int size1, size2,size3,size4;
		int SAMPLE_1,SAMPLE_2,SAMPLE_3,SAMPLE_4;
		unsigned char PLOTTER_BUFFER;
		HAL_UART_Transmit(&huart2,(unsigned char*)"$",1,500); //Start Of Frame
    // Send Negative = -20 //
		SAMPLE_1 = 20;
		size1 = sprintf((char*)&PLOTTER_BUFFER, "%d", SAMPLE_1);
		HAL_UART_Transmit(&huart2,(unsigned char*)"-",1,500);
		HAL_UART_Transmit(&huart2,(unsigned char*)&PLOTTER_BUFFER,size1,500);
		HAL_UART_Transmit(&huart2,(unsigned char*)" ",1,500);
    // Send Negative = -20 //		
		
		//Send Positive = 40 //
		SAMPLE_2 = 40;
	  size2 = sprintf((char*)&PLOTTER_BUFFER, "%d", SAMPLE_2);
		HAL_UART_Transmit(&huart2,(unsigned char*)&PLOTTER_BUFFER,size2,500);
		HAL_UART_Transmit(&huart2,(unsigned char*)" ",1,500);
		//Send Positive = 40 //
		
		//Send Postive and larger than 255 = 320 //
		SAMPLE_3 = 320;
	  size3 = sprintf((char*)&PLOTTER_BUFFER, "%d", SAMPLE_3);
		HAL_UART_Transmit(&huart2,(unsigned char*)&PLOTTER_BUFFER,size3,500);
		HAL_UART_Transmit(&huart2,(unsigned char*)" ",1,500);		
		//Send Postive and larger than 255 = 320 //
		
		//Send Negative and larger than 255 = -270 //
		SAMPLE_4 = 270;
	  size4 = sprintf((char*)&PLOTTER_BUFFER, "%d", SAMPLE_4);
		HAL_UART_Transmit(&huart2,(unsigned char*)"-",1,500);
		HAL_UART_Transmit(&huart2,(unsigned char*)&PLOTTER_BUFFER,size4,500);
		HAL_UART_Transmit(&huart2,(unsigned char*)" ",1,500);		
		//Send Postive and larger than 255 = -270 //		
		HAL_UART_Transmit(&huart2,(unsigned char*)";",1,500); //End Of Frame
		HAL_Delay(600);  //Slow down by the Message output by delay (can be improved by timer interrupt)
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //USART Receive Interrupt
{
  UNUSED(huart);
	if (huart == &huart2)  //Decide what should USART2 do when message is received
	{

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
