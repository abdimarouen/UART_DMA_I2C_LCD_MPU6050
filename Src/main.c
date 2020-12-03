
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include <string.h>

/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "stdio.h" // for printf
/* USER CODE END Includes */

#ifdef __GNUC__

  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private variables ---------------------------------------------------------*/
  
#define SLAVE_ADDRESS_MPU6050 0x68//0x68 // (SLAVE_ADDRESS_MPU6050 << 1) = 0xD0 // 0b11010000
#define MPU6050_WHO_AM_I_REG 0x75

#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void application_handling(char *cmd);// change state according to commands

void MPU6050_Init (void)
{
	uint8_t check, data;
    
    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), MPU6050_WHO_AM_I_REG, 1, &check, 1, 1000); //check MPU
    
    if (check == 0x68)
    {
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), PWR_MGMT_1, 1, &data, 1, 1000); //Wakeup MPU
        data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), SMPLRT_DIV_REG, 1, &data, 1, 1000); //choose rate
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), ACCEL_CONFIG, 1, &data, 1, 1000);//ACC 2g init_config 
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), GYRO_CONFIG, 1, &data, 1, 1000);//Gero 250dps init_config
        
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    
}

void MPU6050_GET_ACC (float* Ax, float* Ay, float* Az) //Get ACC Data
{
    uint8_t data[6];
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    
    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), ACCEL_XOUT_H, 1, &data[0], 6, 1000);
    Accel_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    Accel_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    Accel_Z_RAW = (int16_t)((data[4] << 8) | data[5]);
    
    *Ax = Accel_X_RAW / 16384.0;
    *Ay = Accel_Y_RAW / 16384.0;
    *Az = Accel_Z_RAW / 16384.0;
}

void MPU6050_GET_GERO (float* Gx, float* Gy, float* Gz) //Get Gyro Data
{
    uint8_t data[6];
    int16_t GERO_X_RAW, GERO_Y_RAW, GERO_Z_RAW;
    
    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), GYRO_XOUT_H, 1, &data[0], 6, 1000);
    GERO_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    GERO_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    GERO_Z_RAW = (int16_t)((data[4] << 8) | data[5]);
    
    *Gx = GERO_X_RAW / 131.0;
    *Gy = GERO_Y_RAW / 131.0;
    *Gz = GERO_Z_RAW / 131.0;
}

void LCD_PRINT(const char A, float X, float Y, float Z, int l,int c) //Write to LCD
{
    char str[16];
    char pacc[64];
    
    strcpy (pacc, &A);      
    sprintf(str, "%.2f", X);
    strcat (pacc, str);
    sprintf(str, "%.2f", Y);
    strcat (pacc, "|");
    strcat (pacc, str);
    sprintf(str, "%.2f", Z);
    strcat (pacc, "|");
    strcat (pacc, str);

    lcd_put_cur(l,c);
    lcd_send_string(pacc);
    strcpy(pacc, "");
}
void lcd_HALLO(void)
{
    lcd_clear();
    lcd_put_cur(0,0);
    lcd_send_string("Hello World");
    HAL_Delay(1000);
    lcd_clear();
    lcd_put_cur(1,0);
    lcd_send_string("it's working");
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
float Ax, Ay, Az;
float Gx, Gy, Gz;

char rxbuf[25];

typedef enum
{
	UT_START = 0,
	UT_APP,
}Uart_StateMachine;

Uart_StateMachine myUART_State = UT_START;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_UART4_Init();
  MX_I2C1_Init();
  
  lcd_init();
  MPU6050_Init();
  /* USER CODE BEGIN 2 */
    printf("starting application...\n");
    
    HAL_UART_Receive_DMA(&huart4, (uint8_t*)rxbuf, 3);
    
    //Fixed size commend of 3 bytes *[]#
    
    lcd_HALLO(); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      MPU6050_GET_ACC (&Ax, &Ay, &Az);
      MPU6050_GET_GERO (&Gx, &Gy, &Gz);
    
      lcd_clear();
      LCD_PRINT('A', Ax, Ay, Az, 0,0);
      LCD_PRINT('G', Gx, Gy, Gz, 1,0);

      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
      HAL_Delay(200);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin|LED_ORANGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BOTTON_Pin */
  GPIO_InitStruct.Pin = USER_BOTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BOTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_BLUE_Pin LED_RED_Pin LED_ORANGE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin|LED_ORANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the uart4 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

    switch(myUART_State)
    {
        case UT_START: //Waiting for the wright command that starts with '*' and ends with '#'
            if(rxbuf[0] == '*' && rxbuf[2] == '#')
            {
                //prompt for a command recieve lenghth :: pruf of size
                if(((int)rxbuf[1]-'0') <= sizeof(rxbuf))// ((int)rxbuf[1]-'0') : ascii 6=54 and ascii of 0 = 48 6=54-48
                {
                    //ask of a command of the specified lenght
                    printf("Send CMD: \r\n");
                    printf("Size of rxbuf %lu\n", (int)sizeof(rxbuf));
                    printf("Size of rxbuf[1] %lu\n", ((int)rxbuf[1]-'0'));
                    printf("Send CMD: \r\n");
                    HAL_UART_Receive_DMA(&huart4, (uint8_t*)rxbuf, ((int)rxbuf[1]-'0'));
                    myUART_State = UT_APP;
                }
                else
                {
                    printf("invalid size! \r\n");
                    HAL_UART_Receive_DMA(&huart4, (uint8_t*)rxbuf, 3);
                }
            }
            else
            {
                printf("invalid CMD! \r\n");
                HAL_UART_Receive_DMA(&huart4, (uint8_t*)rxbuf, 3);
            }
            break;
        case UT_APP: //process the command
            application_handling(rxbuf);
            memset(rxbuf, sizeof(rxbuf), 0);
            //get back to state START
            myUART_State = UT_START;
            HAL_UART_Receive_DMA(&huart4, (uint8_t*)rxbuf, 3);
            break;
    }
//    //check if command is wright : *[]#
//    if(rxbuf[0] == '*' && rxbuf[2] == '#') // valid command
//    {
//        //if command is equal to 1 => LED_ORANGE ON : *1#
//        if(rxbuf[1] == '1')
//        {
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
//            printf("LED_ORANGE ON\n");
//            
//        }
//        else if(rxbuf[1] == '0')
//        {
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
//            printf("LED_ORANGE OFF\n");
//        }
//        //if command is equal to 0 => LED_ORANGE OFF : *0#
//    }
//    else // invalid command
//    {
//        printf("invalid command!\n");        
//    }
//    //memset(rxbuf, sizeof(rxbuf),0);
//    HAL_UART_Receive_DMA(&huart4, (uint8_t*)rxbuf, 3);
}

void application_handling(char *cmd)
{
	if(strstr(cmd, "LED ON") != NULL)
	{
		//Turn LED ON
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
		printf("LED_ORANGE ON\r\n");
	}
	else if(strstr(cmd, "LED OFF") != NULL)
	{
		//Turn LED OFF
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
		printf("LED_ORANGE OFF\r\n");
	}
	else
	{
		//Invalid command
		printf("Invalid Command!\r\n");
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
