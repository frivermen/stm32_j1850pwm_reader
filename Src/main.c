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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define tim_counter_on()    TIM17->CR1 |= TIM_CR1_CEN
#define tim_counter_off()    TIM17->CR1 &= ~TIM_CR1_CEN
#define tim_us()            TIM17->CNT
#define tim_int_on()        TIM17->DIER |= TIM_DIER_UIE
#define tim_int_off()       TIM17->DIER &= ~TIM_DIER_UIE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define GAP 2     // us

#define SOF 48
#define SOF_MIN (SOF - GAP)
#define SOF_MAX (SOF + GAP)

#define SOF_H 32 // first half 32us
#define SOF_H_MIN (SOF_H - GAP)
#define SOF_H_MAX (SOF_H + GAP)

#define EOD_H 16 // first half 16us
#define EOD_H_MIN (EOD_H - GAP)
#define EOD_H_MAX (EOD_H + GAP)

#define EOF 72
#define EOF_MIN (EOF - GAP)
#define EOF_MAX (EOF + GAP)

#define BIT_LENGTH 24
#define BIT_LENGTH_MIN (BIT_LENGTH - GAP)
#define BIT_LENGTH_MAX (BIT_LENGTH + GAP)

#define LOW_H 16 // first half 16us
#define LOW_H_MIN (LOW_H - GAP)
#define LOW_H_MAX (LOW_H + GAP)

#define HIGH_H 8 // first half 8us
#define HIGH_H_MIN (HIGH_H - GAP)
#define HIGH_H_MAX (HIGH_H + GAP)

volatile uint16_t rising_marker = 0;   // time marker of rising front 
volatile uint16_t falling_marker = 0;  // time marker of falling front 
volatile uint8_t  byte_buffer = 0;
volatile uint8_t  bit_counter = 0;     // from 0 to 7
volatile uint8_t  data_pointer = 0;
volatile uint8_t  data_ready = 0;      // flag for start uart transmit
volatile uint8_t  data[24] = {0};
static const uint8_t hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7','8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void uart_char(int8_t buf) {                  // it faster then HAL
  while ((USART1->ISR & USART_ISR_TXE) == 0);
  USART1->TDR = (uint8_t) buf;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == GPIO_PIN_1) {                // input connect to PB1
    if (GPIOB->IDR & (1 << 1)) {             // rising front
      rising_marker = tim_us();
      tim_us() = 0;
      if (rising_marker <= SOF_MAX && rising_marker >= SOF_MIN) {  // if time between two rising fronts equal 48us
        if ((falling_marker) > (rising_marker - falling_marker)) { // 32/16 "SOF"
          data[0] = '>';
          data_pointer = 1; 
        }
        else {                                                 // 16/32 "EOD"
          data[data_pointer] = ' ';
          data_pointer++; 
        }
        bit_counter = 0b10000000;
        byte_buffer = 0;
      }
    }
    else {                                                    // falling front
      falling_marker = tim_us();
      if (falling_marker <= HIGH_H_MAX && falling_marker >= HIGH_H_MIN) {     // if time between rising fronts and falling front equal 16us (logic '1')
        byte_buffer |= bit_counter;                                        // else do nothing
      }
      bit_counter >>= 1;
      if (bit_counter == 0) {                                             // if byte have 8 bits
        data[data_pointer] = ' ';
        data[++data_pointer] = hex[byte_buffer >> 4];                       // transpose uint8_t to hex char
        data[++data_pointer] = hex[byte_buffer & 0xF];
        data_pointer++;
        bit_counter = 0b10000000;
        byte_buffer = 0;
      }
      tim_int_on();                                                       // set timer 17 interrupt for watch EOF
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM17) { 
    if (data_pointer) {                                                    // if have some data in buffer
      data[data_pointer] = ' ';
      data[++data_pointer] = '<';
      data_pointer++;
      data_ready = 1;                                                     // set flag for print in main()
    }
    tim_int_off();
  }
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)"j1850 pwm started\r\n", 19);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (data_ready) {
      for (uint8_t i = 0; i < data_pointer; i++) {
        uart_char(data[i]);
        data[i] = 0;
      }
      uart_char('\r');
      uart_char('\n');
      data_ready = 0;
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
