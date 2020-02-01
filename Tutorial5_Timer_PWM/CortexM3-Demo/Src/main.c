/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @author         : Chan Tai Wing Vincent
	* @content        : M3 Demo - Timer Pulse Width Modulation (PWM)
	* @version        : Tutorial 5
  ******************************************************************************
  * @attention
  * This program is written for STM32F103RB Nucleo Board. This program is to 
  * demonstrate how to use generate a PWM with certain duty cycle. "INC" and "DEC"
	* are the commands to change the duty cycles in order to change the brightness
	* of D12 LED.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_BUFFER_SIZE 3
#define TRUE 1
#define FALSE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int LED_ON = FALSE;

unsigned char USART_RX_BUFFER[USART_BUFFER_SIZE];
unsigned char SYSTEM_START[] = "WELCEOM TO STM32F103RBT6 !\r\n";
unsigned char SYSTEM_WATCHDOG_RESET[] = "IWDG reset failed !\r\n";
unsigned char SYSTEM_TIMER_IT[] = "ONE second passed !\r\n";
unsigned char SYSTEM_LED_MAX[] = "LED Brightness increased !\r\n";
unsigned char SYSTEM_LED_MIN[] = "LED Brightness decreased !\r\n";

int TIMER_COUNTER = 0;
int LED_PWM = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
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
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2,SYSTEM_START,sizeof(SYSTEM_START)-1,300); //Ask USART2 to send message and timeout = 300ms
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); //Enable PWM Channel 1 in Timer3
	HAL_UART_Receive_IT(&huart2,USART_RX_BUFFER,sizeof(USART_RX_BUFFER)); //Indicate USART2 to have RX interrupt
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		if(LED_ON == TRUE)       //Check whether "LED" is received
		{
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);                //LED ON
		}else{
			     HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);         //LED OFF
		     }
    if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin) == GPIO_PIN_RESET)       //BUTTON pressed then fall into deadloop
		{
			HAL_UART_Transmit(&huart2,SYSTEM_WATCHDOG_RESET,sizeof(SYSTEM_WATCHDOG_RESET)-1,300);
			while(1);  //Deadloop
		}
		if (LED_PWM == 100)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,600);             // Received "INC" ,output a higher duty cycle
		}else if (LED_PWM == 50)
			    {
						__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,80);       // Received "DEC", output a lower duty cycle
					}
		HAL_IWDG_Refresh(&hiwdg);   //WATCHDOG Reset [within 3.2s]
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1024;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 703;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
		if((USART_RX_BUFFER[0] == 'L')&&(USART_RX_BUFFER[1] == 'E')&&(USART_RX_BUFFER[2] == 'D'))  //Must receive "LED" to change the state of LED
		{
			if(LED_ON == TRUE)
			{
				LED_ON = FALSE;
			}else{
				     LED_ON = TRUE;
			     }
    }
		if((USART_RX_BUFFER[0] == 'I')&&(USART_RX_BUFFER[1] == 'N')&&(USART_RX_BUFFER[2] == 'C'))  //Must receive "INC" to have a higher duty cycle
		{
			LED_PWM = 100;
			HAL_UART_Transmit(&huart2,SYSTEM_LED_MAX,sizeof(SYSTEM_LED_MAX)-1,300);
    }
		if((USART_RX_BUFFER[0] == 'D')&&(USART_RX_BUFFER[1] == 'E')&&(USART_RX_BUFFER[2] == 'C'))  //Must receive "DEC" to have a lower duty cycle
		{
			LED_PWM = 50;
			HAL_UART_Transmit(&huart2,SYSTEM_LED_MIN,sizeof(SYSTEM_LED_MIN)-1,300);
    }	
    HAL_UART_Receive_IT(&huart2,USART_RX_BUFFER,sizeof(USART_RX_BUFFER)); //Indicate USART2 to have RX interrupt		
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
