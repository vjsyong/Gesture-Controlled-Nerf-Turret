/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "MadgwickAHRS.h"
#include "sd_hal_mpu6050.h"

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

USART_HandleTypeDef husart1;

/* USER CODE BEGIN PV */
SD_MPU6050 mpu1;

int16_t g_x, g_y, g_z; 
int16_t a_x, a_y, a_z;
float a_ang;
int16_t m_x, m_y, m_z;
float m_ang;
float offset_val;
int offset_set=0;
float temper;
int16_t gScale = 1;
int16_t aScale = 1;
uint8_t magAddr = 0x1A;
float or0, or1, or2, or3;
int sweepLeft = 1;
float pwm_valY = 100, pwm_valX=100;
float pwm_angX,pwm_angY;
float pwm_prevX,pwm_prevY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void I2C_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
static uint8_t I2C_Read(uint8_t slaveAddr, uint8_t regAddr);
static void MAG_Init(void);
float map_pwm(float offset, float ang);
void user_pwm_setvalueX(uint16_t value);
void user_pwm_setvalueY(uint16_t value);
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
		SD_MPU6050_Result result ;
		uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
		uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); 
	MAG_Init();
	HAL_Delay(500);
	
	if(HAL_I2C_IsDeviceReady(&hi2c2, 0xD0, 2, 100) == HAL_OK ){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); 
	} 
	
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); 
	
	if(HAL_I2C_IsDeviceReady(&hi2c2, 0x1A, 2, 100) == HAL_OK ){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); 
	} 
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//user_pwm_setvalue(250);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*
		user_pwm_setvalueX(pwm_valX);
		if (sweepLeft)
		{
			pwm_valX -= 5;
			if (pwm_valX <= 50){
				sweepLeft = 0;
			}
			
		} else {
			pwm_valX += 5;
			if (pwm_valX >= 250){
				sweepLeft = 1;
			}
		}
		
		HAL_Delay(20);
		*/
		//HAL_Delay(10000);
		
				result = SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
		
		//Read Gyro 
	  //HAL_Delay(500);
	  SD_MPU6050_ReadTemperature(&hi2c2,&mpu1);
	  temper = mpu1.Temperature;
	  SD_MPU6050_ReadGyroscope(&hi2c2,&mpu1);
		
	  g_x = mpu1.Gyroscope_X/gScale;
	  g_y = mpu1.Gyroscope_Y/gScale;
	  g_z = mpu1.Gyroscope_Z/gScale;
		
		SD_MPU6050_ReadAccelerometer(&hi2c2,&mpu1);
	  a_x = mpu1.Accelerometer_X/aScale;
	  a_y = mpu1.Accelerometer_Y/aScale;
	  a_z = mpu1.Accelerometer_Z/aScale;
		a_ang = ((float)a_y / 16384 ) * 90;
		MadgwickAHRSupdateIMU(g_x, g_y, g_z, a_x, a_y, a_z);
		or0 = q0; or1=q1; or2=q2;or3=q3;
		
		//Read Mag
		m_x =  I2C_Read(magAddr, 0x00) | (I2C_Read(magAddr, 0x01)<<8);
		m_y =  I2C_Read(magAddr, 0x02) | (I2C_Read(magAddr, 0x03)<<8);
		m_z =  I2C_Read(magAddr, 0x04) | (I2C_Read(magAddr, 0x05)<<8);
		
		m_ang = atan2f((float)m_y, (float)m_x)*180.0/3.14159265;
		
		if (m_ang < 0.0){
			m_ang += 360.0;
		}
		
		pwm_angY = a_y *90 / 16380;

		if (!offset_set){
			offset_val = m_ang;
			pwm_prevY = pwm_angY;
			offset_set = 1;
		}
		
		
		//pwm_ang = ;
		
		//pwm_angY = -pwm_angY;
		if (pwm_prevY - pwm_angY >3 ||  pwm_prevY - pwm_angY < -3){
			pwm_prevY = m_ang;
					if (pwm_angY < -90){
				user_pwm_setvalueY(50);
			} else if (pwm_angY > 90){
				user_pwm_setvalueY(230);
			} else{
				pwm_valY = (float)(150 + pwm_angY*100.0/90);
				user_pwm_setvalueY(pwm_valY);
			}
		}

		
		pwm_angX = map_pwm(offset_val, m_ang);
		pwm_angX = -pwm_angX;
		if (pwm_angX < -90){
				user_pwm_setvalueX(50);
			} else if (pwm_angX > 90){
				user_pwm_setvalueX(230);
			} else{
				pwm_valX = (float)(150 + pwm_angX*100.0/90);
				user_pwm_setvalueX(pwm_valX);
				pwm_prevX = pwm_valX;
		}
		
		

		/*
		if(pwm_ang > 360)
			pwm_ang -= 360;

		if (pwm_ang > 90 && pwm_ang < 180){
			user_pwm_setvalue(50);
		} else if (pwm_ang >= 180 && pwm_ang < 270){
			user_pwm_setvalue(250);
		} else{
			pwm_val = (float)(150 + pwm_ang*100.0/90);
			user_pwm_setvalue(pwm_val);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim3.Init.Prescaler = 80;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 80;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void I2C_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = regAddr; 
	i2cData[1] = data;  
	HAL_I2C_Master_Transmit(&hi2c2, slaveAddr, i2cData, 2, 100);
}

uint8_t I2C_Read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t i2cData[2];
	i2cData[0] = regAddr;
	HAL_I2C_Master_Transmit(&hi2c2, slaveAddr, i2cData, 1, 100);      
	i2cData[1] = 0x00;      
	HAL_I2C_Master_Receive(&hi2c2, slaveAddr, &i2cData[1], 1, 100);  
	return i2cData[1];
}

void MAG_Init()
{
	I2C_Write(magAddr, 0x0A, 0x80);
	I2C_Write(magAddr, 0x0B, 0x01);
	I2C_Write(magAddr, 0x09, 0x1D);
	
	HAL_Delay(1000);
}

void user_pwm_setvalueX(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  
}


void user_pwm_setvalueY(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);  
}
float map_pwm(float offset, float ang)
{
	//float result = 360 -offset + ang;
	/*
	float result = ang + (int)offset % 180;
	if(result > 360)
		result -= 360;
	if(result > 180)
		result -= 360; 
	*/
	float result = ang -offset;
	
	if (offset < 90 && ang > 270+offset){
		result = -(360 -ang +offset);
	} else if (offset > 270 && ang < offset-270){
		result = 360 - offset + ang;
	} 
	return result;
}
/* USER CO*/
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
