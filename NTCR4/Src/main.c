
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
  * COPYRIGHT(c) 2021 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "LP5018.h"
#include "stdbool.h"
//#include "stm32f0xx_hal_tim.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTimeGet;
RTC_TimeTypeDef sTimeSet;
RTC_AlarmTypeDef sAlarmSet;
RTC_AlarmTypeDef sAlarmGet;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void setTube(int tube);
void setNumber(int num);
void updateTubes(uint8_t H, uint8_t M, uint8_t S);
void updateTime(void);
void setTime(void);
void checkAlarm(void);
void setAlarmTime(void);
void setColor(uint8_t brightness);
void displayColor(uint8_t brightness);
void pollButtons(void);
void numberCycle(void);
void screeeee(bool on);

uint8_t seconds = 57;
uint8_t minutes = 59;
uint8_t hours = 10;
uint8_t ampm;
uint8_t alarmSeconds = 0;
uint8_t alarmMinutes = 0;
uint8_t alarmHours = 12;
uint8_t alarmampm;
uint8_t colour = 6;
uint8_t lightLevel = 0xB0;
uint8_t NC = 0; 				//number cycle number

bool TIME_FLAG = false;			//puts into time set mode
bool ALARM_FLAG = false;		//puts into alarm set mode
bool DISPLAY_FLAG = false;		//puts into display set more
bool UP_FLAG = false;			//for use in modes
bool DOWN_FLAG = false;			//for use in modes
bool RIGHT_FLAG = false;		//for use in modes
bool SET_ALARM_FLAG = false;	//is the alarm armed?
bool PM_FLAG = false;			//is it after noon?
bool PM_ALARM_FLAG = false;		//is it alarm after noon?
bool BLINK_FLAG1 = false;		//permission to blink the LEDs
bool BLINK_FLAG2 = false;		//blink the LEDs
bool POLL_BUTTONS_FLAG = false; //permission to poll buttons
bool NUMBER_CYCLE_FLAG = false; //number cycle at end of hours
bool UPDATE_FLAG = false;

//enum mode{normal, setTime, setAlarm, setColor};
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  configLEDs(&hi2c1, 0xFF);
  setLEDsColor(&hi2c1, 0xF0, 0x00, 0x80, 0x80);
  HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  pollButtons();
	  updateTime();
	  setColor(lightLevel);
	  numberCycle();
	  checkAlarm();

	  //blinking
	  if(!BLINK_FLAG2){
		  lightLevel = 0xB0;
		  displayColor(lightLevel);
	  }else{
		  lightLevel = 0x00;
		  displayColor(lightLevel);
	  }

	  //alarm, time, updating, etc
	  if(!ALARM_FLAG){
		  setTime();
		  updateTubes(hours, minutes, seconds);
	  }else{
		  setAlarmTime();
		  updateTubes(alarmHours, alarmMinutes, alarmSeconds);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin|AM_PM_Pin|TUBE1_Pin|TUBE2_Pin 
                          |TUBE3_Pin|TUBE4_Pin|NUM1_Pin|NUM2_Pin 
                          |TUBE6_Pin|BUZZER_Pin|LED_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NUM3_Pin|NUM4_Pin|NUM5_Pin|NUM6_Pin 
                          |NUM7_Pin|NUM8_Pin|NUM9_Pin|NUM0_Pin 
                          |TUBE5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P_CTRL_Pin AM_PM_Pin TUBE1_Pin TUBE2_Pin 
                           TUBE3_Pin TUBE4_Pin NUM1_Pin NUM2_Pin 
                           TUBE6_Pin BUZZER_Pin LED_A_Pin */
  GPIO_InitStruct.Pin = P_CTRL_Pin|AM_PM_Pin|TUBE1_Pin|TUBE2_Pin 
                          |TUBE3_Pin|TUBE4_Pin|NUM1_Pin|NUM2_Pin 
                          |TUBE6_Pin|BUZZER_Pin|LED_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NUM3_Pin NUM4_Pin NUM5_Pin NUM6_Pin 
                           NUM7_Pin NUM8_Pin NUM9_Pin NUM0_Pin 
                           TUBE5_Pin */
  GPIO_InitStruct.Pin = NUM3_Pin|NUM4_Pin|NUM5_Pin|NUM6_Pin 
                          |NUM7_Pin|NUM8_Pin|NUM9_Pin|NUM0_Pin 
                          |TUBE5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B_ALARM_Pin */
  GPIO_InitStruct.Pin = B_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_DISPLAY_Pin B_TIME_Pin B_UP_Pin B_DOWN_Pin 
                           B_RIGHT_Pin */
  GPIO_InitStruct.Pin = B_DISPLAY_Pin|B_TIME_Pin|B_UP_Pin|B_DOWN_Pin 
                          |B_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if (htim->Instance == TIM2)	//100ms timer
	{
		static uint8_t i = 0;
		i++;
		POLL_BUTTONS_FLAG = true;
		//if we are changing the time do not start the update process
		if(!TIME_FLAG){
			UPDATE_FLAG = true;
		}
		//Blink rate
		if(i == 3 && BLINK_FLAG1){
			BLINK_FLAG2 = !BLINK_FLAG2;
			i=0;
		}else if(!BLINK_FLAG1){
			BLINK_FLAG2 = false;
		}else if(i > 3){
			i = 0;
		}
		//Start the Number cycle
		if(minutes == 59 && seconds == 59){
			NUMBER_CYCLE_FLAG = true;
		}
		if(NUMBER_CYCLE_FLAG){
			NC++;
		}
	}
}

void pollButtons(void){
	static uint8_t i = 0;
	if(POLL_BUTTONS_FLAG){
		if(!HAL_GPIO_ReadPin(GPIOB, B_TIME_Pin)){				//time button goes into time set mode
			if(!TIME_FLAG && !ALARM_FLAG && !DISPLAY_FLAG){
				TIME_FLAG = true;
			}else{
				TIME_FLAG = false;
			}
		}
		if(TIME_FLAG && !HAL_GPIO_ReadPin(GPIOA, B_ALARM_Pin)){		//time button and then alarm button puts into alarm set mode
			//ALARM_FLAG = !ALARM_FLAG;
			if(i == 0){
				ALARM_FLAG = true;
				i++;
			}else{
				ALARM_FLAG = false;
				TIME_FLAG = false;
				i = 0;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOB, B_DISPLAY_Pin)){			//display button goes into display mode
			if(!DISPLAY_FLAG && !ALARM_FLAG && !TIME_FLAG){
				DISPLAY_FLAG = true;
			}else{
				DISPLAY_FLAG = false;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOA, B_ALARM_Pin) && !TIME_FLAG){				//alarm button on its own enables/disables the alarm
			if(!SET_ALARM_FLAG && !TIME_FLAG && !DISPLAY_FLAG){
				SET_ALARM_FLAG = true;
			}else{
				SET_ALARM_FLAG = false;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOB, B_UP_Pin)){					//up
			if((!UP_FLAG && TIME_FLAG) || (!UP_FLAG && DISPLAY_FLAG) || (!UP_FLAG && ALARM_FLAG)){
				UP_FLAG = true;
			}else{
				UP_FLAG = false;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOB, B_DOWN_Pin)){				//down
			if((!DOWN_FLAG && TIME_FLAG) || (!DOWN_FLAG && DISPLAY_FLAG) || (!DOWN_FLAG && ALARM_FLAG)){
				DOWN_FLAG = true;
			}else{
				DOWN_FLAG = false;
			}
		}
		if(!HAL_GPIO_ReadPin(GPIOB, B_RIGHT_Pin)){				//right
			if((!RIGHT_FLAG && TIME_FLAG) || (!RIGHT_FLAG && ALARM_FLAG)){
				RIGHT_FLAG = true;
			}else{
				RIGHT_FLAG = false;
			}
		}
		POLL_BUTTONS_FLAG = false;
	}
}

void setAlarmTime(void){
	static uint8_t tube = 0;

	if(ALARM_FLAG){
		if(RIGHT_FLAG){
			RIGHT_FLAG = false;
			tube++;
			if(tube>2){
				tube = 0;
			}
		}
		configLEDs(&hi2c1, 0x03 << tube*2);

		if(UP_FLAG){
			UP_FLAG = false;
			switch(tube){
			case 0:
				alarmHours += 1;
				if(alarmHours > 12){
					alarmHours = 1;
					PM_ALARM_FLAG = !PM_ALARM_FLAG;
				}
				break;
			case 1:
				alarmMinutes += 1;
				if(alarmMinutes > 59){
					alarmMinutes = 0;
				}
				break;
			case 2:
				alarmSeconds += 1;
				if(alarmSeconds > 59){
					alarmSeconds = 0;
				}
				break;
			default:
				break;
			}

		}else if(DOWN_FLAG){
			DOWN_FLAG = false;
			switch(tube){
			case 0:
				if(alarmHours == 1){
					alarmHours = 13;
					PM_ALARM_FLAG = !PM_ALARM_FLAG;
				}
				alarmHours -= 1;

				break;
			case 1:
				if(alarmMinutes == 0){
					alarmMinutes = 60;
				}
				alarmMinutes -= 1;

				break;
			case 2:
				if(alarmSeconds == 0){
					alarmSeconds = 60;
				}
				alarmSeconds -= 1;
				break;
			default:
				break;
			}
		} else{
			//nothing
		}
		/*sAlarmSet.Seconds = alarmSeconds;
		sAlarmSet.Minutes = alarmMinutes;
		sAlarmSet.Hours = alarmHours;
		HAL_RTC_SetAlarm_IT(&hrtc, &sAlarmSet, RTC_FORMAT_BIN);*/

	}else{
		tube = 0;
		configLEDs(&hi2c1, 0xFF);
	}
}

void checkAlarm(void){
	if(SET_ALARM_FLAG){
		HAL_GPIO_WritePin(GPIOA, LED_A_Pin, GPIO_PIN_SET);	//turn on the LED
		if(hours == alarmHours && minutes == alarmMinutes && seconds == alarmSeconds && ampm == alarmampm && !TIME_FLAG && !ALARM_FLAG){
			//will only occur if not in a setting mode and if the time matches exactly
			//activate alarm
			screeeee(true);
		}
	}else{
		HAL_GPIO_WritePin(GPIOA, LED_A_Pin, GPIO_PIN_RESET);//no alarm LED
		//silence alarm
		screeeee(false);
	}
}

void screeeee(bool on){
	if(on){
		HAL_GPIO_WritePin(GPIOA, BUZZER_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, BUZZER_Pin, GPIO_PIN_RESET);
	}
}

void setColor(uint8_t brightness){
	if(DISPLAY_FLAG == true){
		if(UP_FLAG == true){
			UP_FLAG = false;
			colour++;
			if(colour > 10){
				colour = 1;
			}
		}else if(DOWN_FLAG == true){
			DOWN_FLAG = false;
			colour--;
			if(colour < 1){
				colour = 10;
			}
		}
		displayColor(brightness);
		BLINK_FLAG1 = true;
	}
	else{
		BLINK_FLAG1 = false;
	}
}

void displayColor(uint8_t brightness){
	switch(colour){
	case 1:
		setLEDsColor(&hi2c1, brightness, 0x80, 0x80, 0x80);	//white
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 2:
		setLEDsColor(&hi2c1, brightness, 0x80, 0x00, 0x00);	//red
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 3:
		setLEDsColor(&hi2c1, brightness, 0x80, 0x40, 0x00);	//Orange
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 4:
		setLEDsColor(&hi2c1, brightness, 0x80, 0x80, 0x00);	//yellow
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 5:
		setLEDsColor(&hi2c1, brightness, 0x00, 0x80, 0x00);	//green
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 6:
		setLEDsColor(&hi2c1, brightness, 0x00, 0x80, 0x80);	//teal
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 7:
		setLEDsColor(&hi2c1, brightness, 0x00, 0x00, 0x80);	//blue
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 8:
		setLEDsColor(&hi2c1, brightness, 0x80, 0x00, 0x80);	//purple
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 9:
		setLEDsColor(&hi2c1, brightness, 0x00, 0x00, 0x00);	//leds off
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_SET);
		break;
	case 10:
		setLEDsColor(&hi2c1, brightness, 0x00, 0x00, 0x00);	//all off
		HAL_GPIO_WritePin(GPIOA, P_CTRL_Pin, GPIO_PIN_RESET);
		break;
	default:
		colour = 1;
		break;
	}
}

void numberCycle(void){
	if(NUMBER_CYCLE_FLAG && (NC < 10) && !TIME_FLAG){
		hours = NC*11;
		minutes = NC*11;
		seconds = NC*11;
		updateTubes(hours, minutes, seconds);
	}else{
		NUMBER_CYCLE_FLAG = false;
		NC = 0;
	}
}

void setTime(void){
	//RTC_TimeTypeDef sTimeSet;
	static uint8_t tube = 0;

	if(TIME_FLAG){
		if(RIGHT_FLAG){
			RIGHT_FLAG = false;
			tube++;
			if(tube>2){
				tube = 0;
			}
		}
		configLEDs(&hi2c1, 0x03 << tube*2);

		if(UP_FLAG){
			UP_FLAG = false;
			switch(tube){
			case 0:
				hours += 1;
				if(hours == 12){
					PM_FLAG = !PM_FLAG;
				}
				if(hours > 12){
					hours = 1;
				}
				break;
			case 1:
				minutes += 1;
				if(minutes > 59){
					minutes = 0;
				}
				break;
			case 2:
				seconds += 1;
				if(seconds > 59){
					seconds = 0;
				}
				break;
			default:
				break;
			}

		}else if(DOWN_FLAG){
			DOWN_FLAG = false;
			switch(tube){
			case 0:
				if(hours == 1){
					hours = 13;
				}else if(hours == 12){
					PM_FLAG = !PM_FLAG;
				}
				hours -= 1;

				break;
			case 1:
				if(minutes == 0){
					minutes = 60;
				}
				minutes -= 1;

				break;
			case 2:
				if(seconds == 0){
					seconds = 60;
				}
				seconds -= 1;
				break;
			default:
				break;
			}
		} else{
			//nothing
		}
		sTimeSet.Seconds = seconds;
		sTimeSet.Minutes = minutes;
		sTimeSet.Hours = hours;
		HAL_RTC_SetTime(&hrtc, &sTimeSet, RTC_FORMAT_BIN);

	}else{
		tube = 0;
		configLEDs(&hi2c1, 0xFF);
	}
}

void updateTime(void){
	static uint8_t i = 0;
	if(UPDATE_FLAG){
		RTC_DateTypeDef sDateGet;
		HAL_RTC_GetTime(&hrtc, &sTimeGet, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDateGet, RTC_FORMAT_BIN);
		seconds = sTimeGet.Seconds;
		minutes = sTimeGet.Minutes;
		hours = sTimeGet.Hours;
		ampm = sTimeGet.TimeFormat;
		if(hours == 12 && minutes == 0 && seconds == 0){
			i++;
			if(i > 5){
				PM_FLAG = !PM_FLAG;
				i = 0;
			}
		}
		UPDATE_FLAG = false;
	}
}

void updateTubes(uint8_t H, uint8_t M, uint8_t S){
	setTube(1);
	setNumber(H/10);
	HAL_Delay(1);
	setTube(2);
	setNumber(H%10);
	HAL_Delay(1);
	setTube(3);
	setNumber(M/10);
	HAL_Delay(1);
	setTube(4);
	setNumber(M%10);
	HAL_Delay(1);
	setTube(5);
	setNumber(S/10);
	HAL_Delay(1);
	setTube(6);
	setNumber(S%10);
	HAL_Delay(1);
	/*if((PM_FLAG && !ALARM_FLAG) || (PM_ALARM_FLAG)){
		//50% duty cycle to help resistor
		//HAL_GPIO_TogglePin(GPIOA, AM_PM_Pin);
		setTube(0);
		HAL_GPIO_WritePin(GPIOA, AM_PM_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, AM_PM_Pin, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOA, AM_PM_Pin, GPIO_PIN_RESET);
	}*/

	if((PM_FLAG && !ALARM_FLAG) || (PM_ALARM_FLAG)){
		//50% duty cycle to help resistor
		HAL_GPIO_TogglePin(GPIOA, AM_PM_Pin);
		//HAL_GPIO_WritePin(GPIOA, AM_PM_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, AM_PM_Pin, GPIO_PIN_RESET);
	}
}

void setTube(int tube){
	switch(tube){
	case 1:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_RESET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOA, TUBE1_Pin, GPIO_PIN_RESET);	//ALL OFF
		HAL_GPIO_WritePin(GPIOA, TUBE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, TUBE5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, TUBE6_Pin, GPIO_PIN_RESET);
		break;
	}
}

void setNumber(int num){
	switch(num){
	case 0:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
	case 9:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOB, NUM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, NUM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, NUM9_Pin, GPIO_PIN_RESET);
		break;
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
