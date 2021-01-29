/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define P_CTRL_Pin GPIO_PIN_0
#define P_CTRL_GPIO_Port GPIOA
#define AM_PM_Pin GPIO_PIN_1
#define AM_PM_GPIO_Port GPIOA
#define TUBE1_Pin GPIO_PIN_2
#define TUBE1_GPIO_Port GPIOA
#define TUBE2_Pin GPIO_PIN_3
#define TUBE2_GPIO_Port GPIOA
#define TUBE3_Pin GPIO_PIN_4
#define TUBE3_GPIO_Port GPIOA
#define TUBE4_Pin GPIO_PIN_5
#define TUBE4_GPIO_Port GPIOA
#define NUM1_Pin GPIO_PIN_6
#define NUM1_GPIO_Port GPIOA
#define NUM2_Pin GPIO_PIN_7
#define NUM2_GPIO_Port GPIOA
#define NUM3_Pin GPIO_PIN_0
#define NUM3_GPIO_Port GPIOB
#define NUM4_Pin GPIO_PIN_1
#define NUM4_GPIO_Port GPIOB
#define NUM5_Pin GPIO_PIN_2
#define NUM5_GPIO_Port GPIOB
#define NUM6_Pin GPIO_PIN_10
#define NUM6_GPIO_Port GPIOB
#define NUM7_Pin GPIO_PIN_11
#define NUM7_GPIO_Port GPIOB
#define NUM8_Pin GPIO_PIN_12
#define NUM8_GPIO_Port GPIOB
#define NUM9_Pin GPIO_PIN_13
#define NUM9_GPIO_Port GPIOB
#define NUM0_Pin GPIO_PIN_14
#define NUM0_GPIO_Port GPIOB
#define TUBE5_Pin GPIO_PIN_15
#define TUBE5_GPIO_Port GPIOB
#define TUBE6_Pin GPIO_PIN_8
#define TUBE6_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOA
#define LED_A_Pin GPIO_PIN_12
#define LED_A_GPIO_Port GPIOA
#define B_ALARM_Pin GPIO_PIN_15
#define B_ALARM_GPIO_Port GPIOA
#define B_DISPLAY_Pin GPIO_PIN_3
#define B_DISPLAY_GPIO_Port GPIOB
#define B_TIME_Pin GPIO_PIN_4
#define B_TIME_GPIO_Port GPIOB
#define B_UP_Pin GPIO_PIN_5
#define B_UP_GPIO_Port GPIOB
#define B_DOWN_Pin GPIO_PIN_6
#define B_DOWN_GPIO_Port GPIOB
#define B_RIGHT_Pin GPIO_PIN_7
#define B_RIGHT_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
