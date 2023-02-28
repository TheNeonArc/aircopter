///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under BSD 3-Clause license,
//  * the "License"; You may not use this file except in compliance with the
//  * License. You may obtain a copy of the License at:
//  *                        opensource.org/licenses/BSD-3-Clause
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
//#include "pwm.h"
//#include "sensor_fusion.h"
//#include "quaternion.h"
//#include "millis.h"
//#include "pid.h"
//
////david's code
//#include "dpid.h"
////alexiys code
////#include "Euler.h"
//
//#include <stdio.h>
//
//#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
//#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim3;
//
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART1_UART_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_USART2_UART_Init(void);
///* USER CODE BEGIN PFP */
//
//typedef union
//{
//	float f;
//	struct
//	{
//		unsigned int mantissa : 23;
//		unsigned int exponent : 8;
//		unsigned int sign : 1;
//	} raw;
//	uint8_t i[4];
//} myfloat;
//
//typedef union
//{
//	float f;
//	uint8_t i[4];
//} float_int;
//
////read from IMU, orientation vectors stored in a_u and orient, combined in comp
//void read_IMU(MPU6050* m_hi2c, float gyro_data[3], float accel_data[3], float errAX, float errAY, float errAZ,
//		float errGX, float errGY, float errGZ, uint32_t* tick, vector* a_u, vector* orient, vector* comp);
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//#define PI 3.14159265358
//
//#ifdef __cplusplus
// extern "C" {
//#endif
//
//	int __io_putchar(int ch)
//	{
//	 uint8_t c[1];
//	 c[0] = ch & 0x00FF;
//	 HAL_UART_Transmit(&huart2, &*c, 1, 100);
//	 return ch;
//	}
//
//	int _write(int file,char *ptr, int len)
//	{
//	 int DataIdx;
//	 for(DataIdx= 0; DataIdx< len; DataIdx++)
//	 {
//	 __io_putchar(*ptr++);
//	 }
//	return len;
//	}
//
//#ifdef __cplusplus
//}
//#endif
//
//void read_IMU(MPU6050* m_hi2c, int16_t gyro_data[3], int16_t accel_data[3], float errAX, float errAY, float errAZ,
//		float errGX, float errGY, float errGZ, uint32_t* tick, vector* a_u, vector* orient, vector* comp)
//{
//	//IMU code
//	m_hi2c->read_raw(&gyro_data[0], &gyro_data[1], &gyro_data[2], &accel_data[0], &accel_data[1], &accel_data[2]);
//
//
//
//
//
//	//convert accel data to unit vector
//// 	vector a;
//// 	a.x = accel_data[0] - errAX;
//// 	a.y = accel_data[1] - errAY;
//// 	a.z = accel_data[2] - errAZ;
//// 	vector_normalize(&a, a_u);
////
//// 	//convert gyro data
//// 	vector g;
//// 	uint32_t tickVal = *tick;
//// 	g.x = (gyro_data[0] - errGX) * PI / 180.0 * (millis() - tickVal) / 1000.0;
//// 	g.y = (gyro_data[1] - errGY) * PI / 180.0 * (millis() - tickVal) / 1000.0;
//// 	g.z = (gyro_data[2] - errGZ) * PI / 180.0 * (millis() - tickVal) / 1000.0;
//// 	*tick = millis();
//// 	vector g_u;
//// 	quaternion q;
////	float g_mag = vector_normalize(&g, &g_u);
////	g_u.x *= -1;
////	g_u.y *= -1;
////	g_u.z *= -1;
////	quaternion_create(&g_u, g_mag, &q);
////	quaternion_rotate(orient, &q , orient);
////	vector_normalize(orient, orient);
////
////	//comp filter
////	float alpha = 0.008;//0.01
////	vector comp1;
////	vector_multiply(a_u, alpha, &comp1);
////	vector rotated_comp;
////	quaternion_rotate(comp, &q, &rotated_comp);
////	vector comp2;
////	vector_multiply(&rotated_comp, 1 - alpha, &comp2);
////	vector_add(&comp1, &comp2, comp);
////	vector_normalize(comp, comp);
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART1_UART_Init();
//  MX_TIM1_Init();
//  MX_TIM3_Init();
//  MX_I2C1_Init();
//  MX_USART2_UART_Init();
//  /* USER CODE BEGIN 2 */
//
//  //PWM
//  PWM pwm(&htim1, &htim3);
//  pwm.start();
//  pwm.setduty(0, 0, 0, 0);
//
//  //IMU
//  int16_t gyro_data[3] = {0, 0, 0};
//  int16_t accel_data[3] = {0, 0, 0};
//  MPU6050 m_hi2c(&hi2c1);
//  m_hi2c.start();
//  millis_begin();
//  uint32_t tick = 0;
//  if (HAL_I2C_IsDeviceReady(&hi2c1, 0xD1, 2, HAL_MAX_DELAY) == HAL_OK)
//  {
//  }
//
//  //
//  Orientation orientation;
//
//  vector a_u;
//  a_u.x = 0;
//  a_u.y = 0;
//  a_u.z = 0;
//  vector orient;
//  orient.x = 0;
//  orient.y = 0;
//  orient.z = 1;
//  vector comp;
//  comp.x = 0;
//  comp.y = 0;
//  comp.z = 1;
//  //bias compensation
//  double sumGX = 0;
//  double sumGY = 0;
//  double sumGZ = 0;
//  double sumAX = 0;
//  double sumAY = 0;
//  double sumAZ = 0;
//
//  for (int i = 0; i < 20; i++)
//  {
//	  int16_t gx, gy, gz, ax, ay, az;
//	  m_hi2c.read_raw(&gx, &gy, &gz, &ax, &ay, &az);
//	  sumGX += gx;
//	  sumGY += gy;
//	  sumGZ += gz;
//	  sumAX += ax;
//	  sumAY += ay;
//	  sumAZ += az;
//  }
//  float errGX = (sumGX / 20.0);
//  float errGY = (sumGY / 20.0);
//  float errGZ = (sumGZ / 20.0);
//  float errAX = (sumAX / 20.0);
//  float errAY = (sumAY / 20.0);
//  float errAZ = (sumAZ / 20.0) - 1.0; //base value 1g
//
//  tick = millis();
//
//  uint32_t begintick = millis();
//
//  float u = 20;
//  float m = 2 * u;
//  float minduty = 0;
//
//  //test
////	  float u = 20;
////	  float m = 40;
////	  float minduty = 0;
//
//  //David PID
//  float Kp = 100;//50;
//  float Ki = 0;//5;
//  float Kd = 0;//3000;
//
//
//  DPIDController pidroll = {
//		  Kp / 100 * u, Ki / 100 * u, Kd / 100 * u,
//		  -100, 100,
//		  0.001};
//  DPIDController pidpitch = {
//		  Kp / 100 * u, Ki / 100 * u, Kd / 100 * u,
//		  -100, 100,
//		  0.001};
//  DPIDController_Init(&pidroll);
//  DPIDController_Init(&pidpitch);
//
//
////  PIDController pidroll2 = {
////		  0,0,0,//Kp / 100 * u, Ki / 100 * u, Kd / 100 * u,
////		  0.005,
////		  -u, u,
////		  -u, u,
////		  0.001};
////  PIDController pidpitch2 = {
////		  Kp / 100 * u, Ki / 100 * u, Kd / 100 * u,
////		  0.005,
////		  -u, u,
////		  -u, u,
////		  0.001};
////  PIDController_Init(&pidroll2);
////  PIDController_Init(&pidpitch2);
//
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//	  //PWM code
//	  if (millis() < begintick + 5000)
//	  {
//		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
//		  pwm.setduty(0, 0, 0, 0);
//
//		  HAL_Delay(500);
//		  continue;
//	  }
//	  if (millis() > begintick + 10000)
//	  {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//		  pwm.setduty(0, 0, 0, 0);
//		  HAL_Delay(100000);
//		  continue;
//	  }
//
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//
//	  //read from IMU
//	  read_IMU(&m_hi2c, gyro_data, accel_data, errAX, errAY, errAZ,
//	  		errGX, errGY, errGZ, &tick, &a_u, &orient, &comp);
//
//	  //David PID
//	  float roll = m_hi2c.orientation.roll; //vector_roll(&comp);
//	  float pitch = m_hi2c.orientation.pitch; //vector_pitch(&comp);
//
//	  float r = DPIDController_Update(&pidroll, 0.0f, roll);
//	  float p = DPIDController_Update(&pidpitch, 0.0f, pitch);
//
////	  float r = PIDController_Update(&pidroll2, 0.0f, roll);
////	  float p = PIDController_Update(&pidpitch2, 0.0f, pitch);
//
//
//
//	  float m1  = MAX(minduty,MIN(m,u+r-p));
//	  float m2 = MAX(minduty,MIN(m,u+r+p));
//	  float m3  = MAX(minduty,MIN(m,u-r+p));
//	  float m4 = MAX(minduty,MIN(m,u-r-p));
//
//	  m1 = 5;
//	  m2 = 5;
//	  m3 = 5;
//	  m4 = 5;
//
//
//	  pwm.setduty(m1,m2,m3,m4);
//
//	  //write to UART
////	  float_int ax, ay, az;
////	  ax.f = r;//comp.x;
////	  ay.f = p;//comp.y;
////	  az.f = p;//comp.z;
////	  ax.f = comp.x;
////	  ay.f = comp.y;
////	  az.f = comp.z;
////	  uint8_t intbuf[12];
////	  intbuf[0] = ax.i[0];
////	  intbuf[1] = ax.i[1];
////	  intbuf[2] = ax.i[2];
////	  intbuf[3] = ax.i[3];
////	  intbuf[4] = ay.i[0];
////	  intbuf[5] = ay.i[1];
////	  intbuf[6] = ay.i[2];
////	  intbuf[7] = ay.i[3];
////	  intbuf[8] = az.i[0];
////	  intbuf[9] = az.i[1];
////	  intbuf[10] = az.i[2];
////	  intbuf[11] = az.i[3];
////	  HAL_UART_Transmit(&huart1, intbuf, 12, HAL_MAX_DELAY);
//
//
//
//
//
//
////	  float_int fi;
////
////	  fi.f = r;//MAX(minduty,MIN(m,u+r-p));
////	  HAL_UART_Transmit(&huart1, fi.i, 4, HAL_MAX_DELAY);
////	  HAL_Delay(10);
//
//
//
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief I2C1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C1_Init(void)
//{
//
//  /* USER CODE BEGIN I2C1_Init 0 */
//
//  /* USER CODE END I2C1_Init 0 */
//
//  /* USER CODE BEGIN I2C1_Init 1 */
//
//  /* USER CODE END I2C1_Init 1 */
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 100000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C1_Init 2 */
//
//  /* USER CODE END I2C1_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 3;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 99;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//  HAL_TIM_MspPostInit(&htim1);
//
//}
//
///**
//  * @brief TIM3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM3_Init(void)
//{
//
//  /* USER CODE BEGIN TIM3_Init 0 */
//
//  /* USER CODE END TIM3_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM3_Init 1 */
//
//  /* USER CODE END TIM3_Init 1 */
//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 3;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 99;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */
//
//  /* USER CODE END TIM3_Init 2 */
//  HAL_TIM_MspPostInit(&htim3);
//
//}
//
///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PC8 */
//  GPIO_InitStruct.Pin = GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
