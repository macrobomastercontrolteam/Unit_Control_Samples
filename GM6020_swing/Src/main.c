/* USER CODE BEGIN Header */
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
 * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  TURN_WAIT,
  TURN_INIT,
  TURN_LEFT_INIT,
  TURN_LEFT_IDLE,
  TURN_LEFT,
  TURN_RIGHT_INIT,
  TURN_RIGHT_IDLE,
  TURN_RIGHT,
} edirection_state;
typedef enum
{
  SPEED_STOP_INIT,
  SPEED_STOP,
  SPEED_RAMP,
  SPEED_CONST,
} espeed_state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROTATE_RPM 5U
#define ROTOR_ANGLE_MAX_RAW 8192U
#define ROTATE_HALF_RANGE_RAW (40U * ROTOR_ANGLE_MAX_RAW / 360U)                                      // SWITCH_DIRECTION_DEADBAND_ANGLE/2 < ROTATE_HALF_RANGE_RAW < 180
#define SWITCH_DIRECTION_DEADBAND_ANGLE 15U                                                           // degree
#define SWITCH_DIRECTION_DEADBAND_TIME ((SWITCH_DIRECTION_DEADBAND_ANGLE * 500U) / (3U * ROTATE_RPM)) // milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern moto_info_t motor_info[MOTOR_MAX_NUM];
pid_struct_t motor_pid[7];
float target_speed_abs = 0.0f;
float target_speed = 0.0f;
uint32_t timestamp = 0;
uint16_t target_speed_uint16 = 0;
uint16_t rotor_angle_center = 0;
uint8_t overflow_underflow_flag = 0; // 0 is normal, 1 is underflow, 2 is overflow
uint8_t stop_flag = 1;
uint8_t direction_flag = 0; // 0 is left, 1 is right
espeed_state speed_state = SPEED_STOP_INIT;
edirection_state direction_state = TURN_WAIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void direction_manager(void);
void speed_manager(void);
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
  // pwm_init();                              // start pwm output
  can_user_init(&hcan1); // config can filter, start can
  for (uint8_t i = 0; i < 7; i++)
  {
    pid_init(&motor_pid[i], 40, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
  }
  /* USER CODE END 2 */

  while (1)
  {
    // press user key to toggle stop/restart
    if (key_scan())
    {
      stop_flag = stop_flag ? 0 : 1;
    }

    direction_manager();

    speed_manager();

    if (!stop_flag)
    {
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed, motor_info[0].rotor_speed);
      /* send motor control message through can bus*/
      set_motor_voltage(0, motor_info[0].set_voltage, 0, 0, 0);
    }
    else{
      motor_pid[0].ref=0;
      motor_pid[0].fdb=0;
      motor_pid[0].err[0]=0;
      motor_pid[0].err[1]=0;
      motor_pid[0].p_out=0;
      motor_pid[0].i_out=0;
      motor_pid[0].d_out=0;
      motor_pid[0].output=0;
    }

    /* system delay 1ms */
    HAL_Delay(0);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void speed_manager(void)
{
  switch (speed_state)
  {
  case SPEED_STOP_INIT:
    target_speed = 0.0f;
    target_speed_uint16 = 0;
    speed_state = SPEED_STOP;
  case SPEED_STOP:
    if (!stop_flag)
    {
      speed_state = SPEED_RAMP;
    }
    break;

  case SPEED_RAMP:
    if (stop_flag)
    {
      speed_state = SPEED_STOP_INIT;
    }
    else
    {
      // ramp up speed within 500 ms
      if (target_speed_uint16 < 500U)
      {
        target_speed_uint16++;
        target_speed_abs = ((float)target_speed_uint16) * ROTATE_RPM / 500.0f; // avoid floating point accumulation error
        if (direction_flag)
        {
          target_speed = -1.0f * target_speed_abs; // go right
        }
        else
        {
          target_speed = target_speed_abs; // go left
        }
      }
      else
      {
        speed_state = SPEED_CONST;
      }
    }
    break;
  case SPEED_CONST:
    if (stop_flag)
    {
      speed_state = SPEED_STOP_INIT;
    }
    else if (direction_flag)
    {
      target_speed = -1.0f * target_speed_abs; // go right
    }
    else
    {
      target_speed = target_speed_abs; // go left
    }
    break;
  default:
    Error_Handler();
    break;
  }
}

void direction_manager(void)
{
  if (stop_flag)
  {
    direction_state = TURN_WAIT;
  }

  switch (direction_state)
  {
  case TURN_WAIT:
    if (!stop_flag)
    {
      direction_state = TURN_INIT;
    }
    break;
  case TURN_INIT:
    // wait until first rotor angle signal is received
    if (motor_info[0].rotor_angle != 0)
    {
      // mark inititial position as center
      rotor_angle_center = motor_info[0].rotor_angle;
      if (rotor_angle_center < ROTATE_HALF_RANGE_RAW)
      {
        overflow_underflow_flag = 1; // underflow
      }
      else if (rotor_angle_center > ROTOR_ANGLE_MAX_RAW - ROTATE_HALF_RANGE_RAW)
      {
        overflow_underflow_flag = 2; // overflow
      }
      else
      {
        overflow_underflow_flag = 0; // normal situation
      }
      direction_state = TURN_LEFT_INIT;
    }
    break;

  case TURN_LEFT_INIT:
    direction_flag = 0;
    timestamp = HAL_GetTick();
    direction_state = TURN_LEFT_IDLE;
  case TURN_LEFT_IDLE:
    if (HAL_GetTick() - timestamp > SWITCH_DIRECTION_DEADBAND_TIME)
    {
      direction_state = TURN_LEFT;
    }
    break;
  case TURN_LEFT:
    // target_speed > 0, and rotor_angle increase
    // target_speed = target_speed > 0 ? target_speed : -target_speed;
    switch (overflow_underflow_flag)
    {
    case 0:
      if (motor_info[0].rotor_angle > rotor_angle_center + ROTATE_HALF_RANGE_RAW)
      {
        direction_state = TURN_RIGHT_INIT;
      }
      break;
    case 1:
      // underflow
      if ((motor_info[0].rotor_angle > rotor_angle_center + ROTATE_HALF_RANGE_RAW) && (motor_info[0].rotor_angle < ROTOR_ANGLE_MAX_RAW - (ROTATE_HALF_RANGE_RAW - rotor_angle_center)))
      {
        direction_state = TURN_RIGHT_INIT;
      }
      break;
    case 2:
      // overflow
      if ((motor_info[0].rotor_angle < rotor_angle_center - ROTATE_HALF_RANGE_RAW) && (motor_info[0].rotor_angle > rotor_angle_center + ROTATE_HALF_RANGE_RAW - ROTOR_ANGLE_MAX_RAW))
      {
        direction_state = TURN_RIGHT_INIT;
      }
      break;
    default:
      Error_Handler();
      break;
    }
    break;

  case TURN_RIGHT_INIT:
    direction_flag = 1;
    timestamp = HAL_GetTick();
    direction_state = TURN_RIGHT_IDLE;
  case TURN_RIGHT_IDLE:
    if (HAL_GetTick() - timestamp > SWITCH_DIRECTION_DEADBAND_TIME)
    {
      direction_state = TURN_RIGHT;
    }
    break;
  case TURN_RIGHT:
    // target_speed = target_speed > 0 ? -target_speed : target_speed;
    switch (overflow_underflow_flag)
    {
    case 0:
      if (motor_info[0].rotor_angle < rotor_angle_center - ROTATE_HALF_RANGE_RAW)
      {
        direction_state = TURN_LEFT_INIT;
      }
      break;
    case 1:
      // underflow
      if ((motor_info[0].rotor_angle > rotor_angle_center + ROTATE_HALF_RANGE_RAW) && (motor_info[0].rotor_angle < ROTOR_ANGLE_MAX_RAW - (ROTATE_HALF_RANGE_RAW - rotor_angle_center)))
      {
        direction_state = TURN_LEFT_INIT;
      }
      break;
    case 2:
      // overflow
      if ((motor_info[0].rotor_angle < rotor_angle_center - ROTATE_HALF_RANGE_RAW) && (motor_info[0].rotor_angle > rotor_angle_center + ROTATE_HALF_RANGE_RAW - ROTOR_ANGLE_MAX_RAW))
      {
        direction_state = TURN_LEFT_INIT;
      }
      break;
    default:
      Error_Handler();
      break;
    }
    break;
  default:
    Error_Handler();
    break;
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
