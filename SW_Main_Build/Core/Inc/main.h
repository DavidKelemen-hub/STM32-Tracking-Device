/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define In4_Pin GPIO_PIN_2
#define In4_GPIO_Port GPIOE
#define Lights_LowBeam_Top_Right_Pin GPIO_PIN_8
#define Lights_LowBeam_Top_Right_GPIO_Port GPIOE
#define Lights_Pos_Left_Pin GPIO_PIN_11
#define Lights_Pos_Left_GPIO_Port GPIOE
#define Lights_LowBeam_Top_Left_Pin GPIO_PIN_13
#define Lights_LowBeam_Top_Left_GPIO_Port GPIOE
#define Lights_LowBeam_Bottom_Left_Pin GPIO_PIN_14
#define Lights_LowBeam_Bottom_Left_GPIO_Port GPIOE
#define Lights_HighBeam_Pin GPIO_PIN_10
#define Lights_HighBeam_GPIO_Port GPIOB
#define Button_Doors_Pin GPIO_PIN_11
#define Button_Doors_GPIO_Port GPIOB
#define In3_Pin GPIO_PIN_11
#define In3_GPIO_Port GPIOD
#define Button_Start_Pin GPIO_PIN_8
#define Button_Start_GPIO_Port GPIOC
#define Fan_Relay_Pin GPIO_PIN_1
#define Fan_Relay_GPIO_Port GPIOD
#define Switch_AUTO_Pin GPIO_PIN_3
#define Switch_AUTO_GPIO_Port GPIOD
#define Switch_HIGHBEAM_Pin GPIO_PIN_4
#define Switch_HIGHBEAM_GPIO_Port GPIOD
#define Switch_LOWBEAM_Pin GPIO_PIN_5
#define Switch_LOWBEAM_GPIO_Port GPIOD
#define Switch_POS_Pin GPIO_PIN_6
#define Switch_POS_GPIO_Port GPIOD
#define Switch_OFF_Pin GPIO_PIN_7
#define Switch_OFF_GPIO_Port GPIOD
#define Touch_Sensor_Pin GPIO_PIN_9
#define Touch_Sensor_GPIO_Port GPIOG
#define Button_Fan_Pin GPIO_PIN_12
#define Button_Fan_GPIO_Port GPIOG
#define Lights_Pos_Right_Pin GPIO_PIN_6
#define Lights_Pos_Right_GPIO_Port GPIOB
#define Lights_LowBeam_Bottom_Right_Pin GPIO_PIN_7
#define Lights_LowBeam_Bottom_Right_GPIO_Port GPIOB
#define Green_Pin GPIO_PIN_8
#define Green_GPIO_Port GPIOB
#define Blue_Pin GPIO_PIN_9
#define Blue_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
