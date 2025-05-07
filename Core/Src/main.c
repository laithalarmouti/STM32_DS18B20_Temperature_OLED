/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body using SSD1306 OLED display and DS18B20 sensor
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
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
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Delay function using TIM11 */
void delay (uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim11, 0);
    while ((__HAL_TIM_GET_COUNTER(&htim11)) < time);
}

/* Display temperature on the SSD1306 OLED */
void Display_Temp (float Temp)
{
    char str[20] = {0};
    ssd1306_SetCursor(0, 0);
    sprintf(str, "TEMP:- %.2f C", Temp);
    ssd1306_WriteString(str, Font_7x10, White);
    ssd1306_UpdateScreen();
}

/* DS18B20 functions and pin configuration */

#define DS18B20_PORT 	GPIOA
#define DS18B20_PIN 	GPIO_PIN_1

/* Configure pin as output */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/* Configure pin as input */
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/* DS18B20 Start signal */
uint8_t DS18B20_Start(void)
{
    uint8_t Response = 0;
    Set_Pin_Output(DS18B20_PORT, DS18B20_PIN); // set the pin as output
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull the pin low
    delay(480); // delay according to datasheet

    Set_Pin_Input(DS18B20_PORT, DS18B20_PIN); // set the pin as input
    delay(80); // delay according to datasheet

    if (!(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)))
        Response = 1; // presence pulse detected
    else
        Response = 0;

    delay(400); // complete the initialization delay

    return Response;
}

/* DS18B20 Write byte */
void DS18B20_Write(uint8_t data)
{
    Set_Pin_Output(DS18B20_PORT, DS18B20_PIN); // set as output

    for (int i = 0; i < 8; i++)
    {
        if ((data & (1 << i)) != 0) // if the bit is high
        {
            // write 1
            Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
            HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull low
            delay(1); // wait 1 µs
            Set_Pin_Input(DS18B20_PORT, DS18B20_PIN); // release bus
            delay(60);
        }
        else // if the bit is low
        {
            // write 0
            Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
            HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0); // pull low
            delay(60);
            Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
        }
    }
}

/* DS18B20 Read byte */
uint8_t DS18B20_Read(void)
{
    uint8_t value = 0;
    Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

    for (int i = 0; i < 8; i++)
    {
        Set_Pin_Output(DS18B20_PORT, DS18B20_PIN); // drive line low
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
        delay(2); // wait 2 µs

        Set_Pin_Input(DS18B20_PORT, DS18B20_PIN); // release bus
        if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))
        {
            value |= 1 << i; // read a '1'
        }
        delay(60);
    }
    return value;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  float Temperature = 0;
  uint8_t Temp_byte1 = 0, Temp_byte2 = 0;
  uint16_t TEMP = 0;
  uint8_t Presence = 0;
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
  MX_I2C1_Init();
  MX_TIM11_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim11);

  // Initialize the SSD1306 OLED display
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* DS18B20 Temperature Conversion */
    Presence = DS18B20_Start();
    HAL_Delay(1);
    DS18B20_Write(0xCC); // Skip ROM command
    DS18B20_Write(0x44); // Start temperature conversion
    HAL_Delay(800);      // Wait for conversion

    Presence = DS18B20_Start();
    HAL_Delay(1);
    DS18B20_Write(0xCC); // Skip ROM command
    DS18B20_Write(0xBE); // Read scratchpad

    Temp_byte1 = DS18B20_Read();
    Temp_byte2 = DS18B20_Read();
    TEMP = (Temp_byte2 << 8) | Temp_byte1;
    Temperature = (float)TEMP / 16;

    // Display the temperature on the OLED
    Display_Temp(Temperature);

    HAL_Delay(3000);
  }
  /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 4;
  RCC_OscInitStruct.PLL.PLLN       = 50;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                     |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{
  htim11.Instance               = TIM11;
  htim11.Init.Prescaler         = 50 - 1;
  htim11.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim11.Init.Period            = 0xffff - 1;
  htim11.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* Configure GPIO pin Output Level for DS18B20 pin and LED */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | LD2_Pin, GPIO_PIN_RESET);

  /* Configure user button pin if needed (example: B1_Pin) */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure DS18B20 pin and LD2 as output (DS18B20 data line will be reconfigured dynamically) */
  GPIO_InitStruct.Pin   = GPIO_PIN_1 | LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Additional GPIO configuration (e.g., USART) if required */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
  // User can add his own implementation to report the file name and line number.
}
#endif /* USE_FULL_ASSERT */
