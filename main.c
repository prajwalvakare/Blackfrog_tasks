/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FLASH_CS_PORT    GPIOA
#define FLASH_CS_PIN     GPIO_PIN_4
#define flash_cs_low()  HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET)      //Pulls CS pin LOW → Selects (activates) the SPI Flash
#define flash_cs_high() HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET)		//Pulls CS pin HIGH → Deselects (deactivates) the SPI Flash
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct __attribute__((packed))
{
	uint8_t temperature;
    uint8_t pwm;
} FlashData;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t flash_read_id(void);
float flash_read_temp(void);
void flash_write_data(float temp, uint8_t pwm);
int process_temperature(void);
void flash_read_data();
void flash_erase_block(uint16_t page);
uint8_t flash_is_busy(void);
void flash_wait_busy(void);
void flash_page_read(uint16_t page);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buf[6];
uint8_t cmd = 0xFD; // High precision measurement command
uint16_t raw_temp;
float temp_c;
float temp_tx;
int decimal;
int integrity;
char msg[50];
char msg_2[30];
uint8_t fan_percent = 0;
uint32_t id;
uint32_t flash_id;
float last_temp_stored = 0.0f;
FlashData data;
float actual_temp;
void display(void);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  flash_cs_high();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  temp_tx = process_temperature();
	  if(temp_tx != -1)
	  {
		  temp_c = temp_tx; // PWM Control Logic
		  if (temp_tx < 25) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 51);
		  	  fan_percent = 20;
		  }
		  else if (temp_tx <= 30) { // No need for >=25, the 'if' handled <25
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 127);
			  fan_percent = 50;
		  }
		  else {
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 204);
			  fan_percent = 80;
		  }
		  flash_read_data();
		  flash_write_data(temp_tx, fan_percent);
	  } // Call display to show status
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 12;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print_uart(char *msg) {
    if(msg != NULL)
    	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

int process_temperature(void) {
    HAL_StatusTypeDef ret;

    // SHT3x Sensor Address 0x44 << 1
    ret = HAL_I2C_Master_Transmit(&hi2c1, (0x44 << 1), &cmd, 1, 100);
    if (ret != HAL_OK) return -1;
    HAL_Delay(20); // Wait for measurement
    ret = HAL_I2C_Master_Receive(&hi2c1, (0x44 << 1), buf, 6, 100);

    if (ret != HAL_OK) return -1;
    raw_temp = (uint16_t)((buf[0] << 8) | buf[1]);
    actual_temp = -45.0f + (175.0f * (float)raw_temp / 65535.0f);
    return (int)actual_temp;
}

void flash_write_enable(void) {
    uint8_t cmd = 0x06;
    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    flash_cs_high();
    HAL_Delay(1);
}

void flash_erase_block(uint16_t page)
{
    flash_write_enable();

    uint8_t cmd[4]; // Increased to 4 bytes
    cmd[0] = 0xD8;                   // Block Erase Command
    cmd[1] = 0x00;                   // DUMMY BYTE (Required for W25N series)
    cmd[2] = (page >> 8) & 0xFF;     // Page Address High
    cmd[3] = page & 0xFF;            // Page Address Low

    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY); // Transmit 4 bytes
    flash_cs_high();
}

uint32_t flash_read_id(void)
{
    uint8_t tx_data = 0x9F;
    uint8_t rx_data[5] = {0}; // Buffer for Command + Dummy + 3 Data bytes

    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, rx_data, 4, HAL_MAX_DELAY);
    flash_cs_high();
    return ((uint32_t)rx_data[1] << 16) | ((uint32_t)rx_data[2] << 8) | rx_data[3];
}
void flash_write_data(float temp, uint8_t pwm)
{
    char msg_buf[100];

    uint32_t flash_id = flash_read_id();
    snprintf(msg_buf, sizeof(msg_buf), "--- System Status ---\r\nSPI Flash ID: %06X\r\n", (unsigned int)flash_id);
    print_uart(msg_buf);

    uint16_t page = 0;     // Page address
    uint16_t column = 0;   // Column inside page
    flash_erase_block(page);

    flash_write_enable();

    data.temperature = temp;
    data.pwm = pwm;

    uint8_t load_cmd[3];
    load_cmd[0] = 0x02;
    load_cmd[1] = (column >> 8) & 0xFF;
    load_cmd[2] = column & 0xFF;

    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, load_cmd, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, sizeof(FlashData), HAL_MAX_DELAY);
    flash_cs_high();

    flash_write_enable();

//    uint8_t exec_cmd[3];
//    exec_cmd[0] = 0x10;
//    exec_cmd[1] = (page >> 8) & 0xFF;
//    exec_cmd[2] = page & 0xFF;
//    0x02 → “Put data in temporary RAM”
//    0x10 → “Burn it permanently into flash”
    uint8_t exec_cmd[4] = { 0x10, 0x00, (page >> 8) & 0xFF, page & 0xFF }; // Add 0x00 dummy
    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, exec_cmd, 4, HAL_MAX_DELAY); // Send 4 bytes
    flash_cs_high();

    while(flash_is_busy());

    print_uart("Flash Write: OK\r\n");
//    flash_read_data();
}
void flash_page_read(uint16_t page)
{
    uint8_t cmd[4];

    cmd[0] = 0x13;             // Page Data Read
    cmd[1] = 0x00;             // Dummy byte
    cmd[2] = (page >> 8) & 0xFF;
    cmd[3] = page & 0xFF;

    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    flash_cs_high();

    while(flash_is_busy());
}

void flash_read_data()
{
    uint8_t cmd[4] = {0x03, 0x00, 0x00, 0x00};
    flash_cs_low();
    HAL_SPI_Transmit(&hspi1, cmd,4, 100);
    HAL_SPI_Receive(&hspi1, (uint8_t*)&data, sizeof(FlashData), 100);
    flash_cs_high();
    char data_read[64] = {0};
    HAL_Delay(10);
    snprintf(data_read, sizeof(data_read),"Flash Read: %d °C, PWM=%d %%\r\n", data.temperature, data.pwm);
    print_uart(data_read);
}

uint8_t flash_is_busy(void)
{
    uint8_t tx[3] = {0x0F, 0xC0, 0xFF};
    uint8_t rx[3] = {0};

    flash_cs_low();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
    flash_cs_high();

    return (rx[2] & 0x01);   // BUSY bit
}

void flash_wait_busy(void)
{
    uint8_t cmd[2];
    uint8_t status;

    cmd[0] = 0x0F;   // Get Feature command
    cmd[1] = 0xC0;   // Status Register address

    do
    {
        flash_cs_low();

        HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);
        HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);

        flash_cs_high();

    } while (status & 0x01);   // Check BUSY bit (bit 0)
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
