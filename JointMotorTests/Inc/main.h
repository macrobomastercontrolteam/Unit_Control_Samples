/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PI 3.14159265358979f
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifndef rad_format
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
#endif /* rad_format */

#define GIMBAL_JOINT_0_ANGLE_MIN (-PI)
#define GIMBAL_JOINT_0_ANGLE_MAX PI
#define GIMBAL_JOINT_0_ANGLE_REST 0.0f
#define GIMBAL_JOINT_0_RC_SEN ((GIMBAL_JOINT_0_ANGLE_MAX - GIMBAL_JOINT_0_ANGLE_MIN) / JOYSTICK_FULL_RANGE)

#define GIMBAL_JOINT_1_ANGLE_MIN (-30.0f / 180.0f * PI)
#define GIMBAL_JOINT_1_ANGLE_MAX (35.0f / 180.0f * PI)
#define GIMBAL_JOINT_1_ANGLE_REST GIMBAL_JOINT_1_ANGLE_MAX
#define GIMBAL_JOINT_1_RC_SEN ((GIMBAL_JOINT_1_ANGLE_MAX - GIMBAL_JOINT_1_ANGLE_MIN) / JOYSTICK_HALF_RANGE)

#define GIMBAL_JOINT_2_ANGLE_MIN (-120.0f / 180.0f * PI)
#define GIMBAL_JOINT_2_ANGLE_MAX 0.0f
#define GIMBAL_JOINT_2_ANGLE_REST GIMBAL_JOINT_2_ANGLE_MIN
#define GIMBAL_JOINT_2_RC_SEN ((GIMBAL_JOINT_2_ANGLE_MAX - GIMBAL_JOINT_2_ANGLE_MIN) / JOYSTICK_HALF_RANGE)

#define GIMBAL_JOINT_3_ANGLE_MIN (-80.0f / 180.0f * PI)
#define GIMBAL_JOINT_3_ANGLE_MAX (80.0f / 180.0f * PI)
#define GIMBAL_JOINT_3_ANGLE_REST 0.0f
#define GIMBAL_JOINT_3_RC_SEN ((GIMBAL_JOINT_3_ANGLE_MAX - GIMBAL_JOINT_3_ANGLE_MIN) / JOYSTICK_FULL_RANGE)

#define GIMBAL_JOINT_4_ANGLE_MIN (-0.5f * PI)
#define GIMBAL_JOINT_4_ANGLE_MAX (0.5f * PI)
#define GIMBAL_JOINT_4_ANGLE_REST 0.0f
#define GIMBAL_JOINT_4_RC_SEN ((GIMBAL_JOINT_4_ANGLE_MAX - GIMBAL_JOINT_4_ANGLE_MIN) / JOYSTICK_FULL_RANGE)

#define GIMBAL_JOINT_5_ANGLE_MIN (10.0f / 180.0f * PI)
#define GIMBAL_JOINT_5_ANGLE_MAX (170.0f / 180.0f * PI)
#define GIMBAL_JOINT_5_ANGLE_REST (90.0f / 180.0f * PI)
#define GIMBAL_JOINT_5_RC_SEN ((GIMBAL_JOINT_5_ANGLE_MAX - GIMBAL_JOINT_5_ANGLE_MIN) / JOYSTICK_FULL_RANGE)

#define GIMBAL_JOINT_6_ANGLE_MIN (-PI)
#define GIMBAL_JOINT_6_ANGLE_MAX PI
#define GIMBAL_JOINT_6_ANGLE_REST 0.0f
#define GIMBAL_JOINT_6_RC_SEN ((GIMBAL_JOINT_6_ANGLE_MAX - GIMBAL_JOINT_6_ANGLE_MIN) / JOYSTICK_FULL_RANGE)

typedef struct
{
  // range is [-PI, PI]
  fp32 joint_target_pos[7];
} engineer_robot_t;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
/* USER CODE BEGIN EFP */
extern engineer_robot_t engineer_robot;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
