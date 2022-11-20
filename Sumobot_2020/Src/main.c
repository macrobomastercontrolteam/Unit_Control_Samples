/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PID_TypeDef motor_pid[4];
int32_t set_spd = 0;
static int key_sta = 0;
int speed_step_sign = +1;

uint32_t counter = 0; 
uint32_t counter1 = 0; 
uint32_t counter2 = 0; 
//uint16_t sonic = 1;
uint16_t TIM_COUNT[2];


/***** State Machine Variables *****/
uint8_t state = 0;
uint8_t ground_state = 0;

uint8_t state0_d = 0;
uint8_t state1_d = 0;
uint8_t state2_d = 0;
uint8_t state3_d = 0;
uint8_t state4_d = 0;
uint8_t state5_d = 0;
uint8_t state6_d = 0;
uint8_t state7_d = 0;
uint8_t state8_d = 0;
uint8_t state9_d = 0;


/***** Ultrasonic variables *****/
uint8_t front1_detect = 0;
uint8_t front2_detect = 0;

float front_sonic_threshold = 80;
float left_sonic_threshold = 50;
float right_sonic_threshold = 50;

const float speedOfSound = 0.0343/2;
float distance_front1;
float distance_front2;
float distance_left;
float distance_right;

uint8_t sonic_last = 0;

/***** Ground Sensor Variable *****/
bool ground_front_detect;
bool ground_left_detect;
bool ground_right_detect;


/****** Motor Variables ******/
//left wheel - Motor[0] right wheel - Motor[1]
int left_speed_sign = +1;
int right_speed_sign = -1;
int32_t wheel_speed_super_slow = 1600;
int32_t wheel_speed_slow = 2000;
int32_t wheel_speed_fast = 8000;
int32_t wheel_speed_medium = 4000;
int32_t wheel_speed_turn = 6000;

uint32_t straight_tick = 2000;
uint32_t fast_90_tick = 250;
uint32_t medium_90_tick = 360;
uint32_t slow_90_tick = 1000;

#define SpeedStep 500


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void usDelay(uint32_t uSec);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Key_Scan(){
		
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET){
						
			if(key_sta == 0){
					
				key_sta = 1;
				
				set_spd += SpeedStep*speed_step_sign;
				
				if(set_spd>8000)
				{
					speed_step_sign = -1;
				}
				
				if(set_spd<=0){
						
					set_spd = 0;
					speed_step_sign = 1;
					
				}
					
			}
			
		}else{
			
			key_sta = 0;
		
		}
		
	
}
/* USER CODE END 0 */




int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t numTicks = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	MX_USART1_UART_Init(); 
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
	MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //启动串口接收

  HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	
	/*< 初始化PID参数 >*/
  for(int i=0; i<4; i++)
  {	
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,1.5,0.1,0); 
  }
	
	//set_spd = 100;
	

  /* USER CODE END 2 */
	HAL_GPIO_WritePin(PWRL_GPIO_Port, PWRL_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(PWRR_GPIO_Port, PWRR_Pin, GPIO_PIN_SET);
	HAL_Delay(4000);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
	

	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
//    if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){   //如果500ms都没有收到遥控器数据，证明遥控器可能已经离线，切换到按键控制模式。
//      Key_Scan();
//    }else{		
//      set_spd = remote_control.ch4*8000/660;
//    }
	
	//	Key_Scan();
		
		
		
		/******** Scan for white line *********/
			
		
		
			
			if ((HAL_GPIO_ReadPin(GROUND1_GPIO_Port, GROUND1_Pin) == GPIO_PIN_RESET 
					|| HAL_GPIO_ReadPin(GROUND2_GPIO_Port, GROUND2_Pin) == GPIO_PIN_RESET 
					|| HAL_GPIO_ReadPin(GROUND3_GPIO_Port, GROUND3_Pin) == GPIO_PIN_RESET) && state != 7) {
				
				
						ground_front_detect = HAL_GPIO_ReadPin(GROUND1_GPIO_Port, GROUND1_Pin) == GPIO_PIN_RESET;
						ground_left_detect = HAL_GPIO_ReadPin(GROUND2_GPIO_Port, GROUND2_Pin) == GPIO_PIN_RESET;
						ground_right_detect = HAL_GPIO_ReadPin(GROUND3_GPIO_Port, GROUND3_Pin) == GPIO_PIN_RESET;
						
						state = 7;
				
			}
			
			if (state == 7){
				
				if (state7_d == 0){
					
					counter = HAL_GetTick();
					state7_d = 1;
					
					if (ground_front_detect){ //if front detected, back up first
							
							ground_state = 0;
							motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_medium;
							motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_medium;
						
							if (ground_left_detect && ground_right_detect) ground_state = 0;
							else if (ground_left_detect) ground_state = 1;
							else if (ground_right_detect) ground_state = 2;
								
					}
					else{ //front not detected
						
							if (ground_left_detect){ //turn right immediately
								ground_state = 3;
								motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_medium;
								motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_medium;
								
							}
						
							else if (ground_right_detect){ //turn left immediately
								ground_state = 4;
								motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_medium;
								motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_medium;
							
							}
							
					}
					
				}
				else if (state7_d == 1){
				
					if (ground_state == 0){// front only
							if (HAL_GetTick() - counter > 500 && HAL_GetTick() - counter < 500 + 1.2 * medium_90_tick){ //turn around 180 
								motor_pid[0].target = left_speed_sign * wheel_speed_turn;
								motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_turn;
							}
							else if (HAL_GetTick() - counter >= 500 + 1.2 * medium_90_tick){
								state7_d = 0;
								state = 1;
							}
						
					}
					else if (ground_state == 1){// front and left
					
							if (HAL_GetTick() - counter > 500 && HAL_GetTick() - counter < 500 + medium_90_tick){ //turn right 90
								motor_pid[0].target = left_speed_sign * wheel_speed_turn;
								motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_turn;
							}
							else if (HAL_GetTick() - counter >= 500 + medium_90_tick){
								state7_d = 0;
								state = 1;
							}
					
					}
					else if (ground_state == 2){// front and right
							if (HAL_GetTick() - counter > 500 && HAL_GetTick() - counter < 500 + medium_90_tick){ //turn left 90 
								motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_turn;
								motor_pid[1].target = right_speed_sign * wheel_speed_turn;
							}
							else if (HAL_GetTick() - counter >= 500 + medium_90_tick){
								state7_d = 0;
								state = 1;
							}
					
					}
					else if (ground_state == 3){// left only
							if (HAL_GetTick() - counter > 300 && HAL_GetTick() - counter < 300 + medium_90_tick + 100){ //turn right 90 
								motor_pid[0].target = left_speed_sign * wheel_speed_turn;
								motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_medium;
							}
							else if (HAL_GetTick() - counter >= 300 + medium_90_tick + 100){
								state7_d = 0;
								state = 1;
							}
					
					}
					else if (ground_state == 4){// right only
					
							if (HAL_GetTick() - counter > 300 && HAL_GetTick() - counter < 300 + medium_90_tick + 100){ //turn left 90 
								motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_medium;
								motor_pid[1].target = right_speed_sign * wheel_speed_turn;
							}
							else if (HAL_GetTick() - counter >= 300 + medium_90_tick + 100){
								state7_d = 0;
								state = 1;
							}
					
					}
			
						
					
				}
	
				
			}

		/******** State Machine & Set Motor Speed *******/
		
		
		////
		
		
			/* state 0 */
			if (state == 0){
			
				if(state0_d == 0){
					counter = HAL_GetTick();
					motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_fast;
					motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_fast;
					state0_d = 1;
			
				}

				else if(state0_d == 1){
					
					if (HAL_GetTick() - counter >= 50 && HAL_GetTick() - counter <= 60){
						motor_pid[0].target = 0;
						motor_pid[1].target = 0;
					}
					else if (HAL_GetTick() - counter > 60){
						state = 1;
						state0_d = 0;
					
					}
					
				
				}
			
			
			}
			
			
			
			/* state 1 */
			if(state == 1){ //scan for enemies
				if (state1_d == 0){

					counter = HAL_GetTick();
					state1_d = 1;
					
				}
				
				else if (state1_d == 1){
					
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
					
					if (HAL_GetTick() - counter <= slow_90_tick){//turn left 90
						motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_super_slow;
						motor_pid[1].target = right_speed_sign * wheel_speed_super_slow;
					}
					else if (HAL_GetTick() - counter > slow_90_tick && HAL_GetTick() - counter < 3 * slow_90_tick){//turn right 180
						motor_pid[0].target = left_speed_sign * wheel_speed_super_slow;
						motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_super_slow;
					}
					else if (HAL_GetTick() - counter >= 3 * slow_90_tick && HAL_GetTick() - counter < 4 * slow_90_tick){//turn left 90
						motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_super_slow;
						motor_pid[1].target = right_speed_sign * wheel_speed_super_slow;
					}
						//state1_d = 2;
						
					else if(HAL_GetTick() - counter >= 4 * slow_90_tick && HAL_GetTick() - counter < 4 * slow_90_tick + straight_tick){
						//state = 2;
						motor_pid[0].target = left_speed_sign * wheel_speed_slow;
						motor_pid[1].target = right_speed_sign * wheel_speed_slow;
					
					}
					
					else if(HAL_GetTick() - counter >= 4 * slow_90_tick + straight_tick ){
						state1_d = 0; 
					
					}
					
				}
			}
			
			/* state 3 */
			else if(state == 3){ //full speed
				
					motor_pid[0].target = left_speed_sign * wheel_speed_fast;
					motor_pid[1].target = right_speed_sign * wheel_speed_fast;
			
			}
			
			/* state 4 */
			else if (state == 4){ //turn left 90 degree
				if (state4_d == 0)
				{
					counter = HAL_GetTick();
					motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_turn;
					motor_pid[1].target = right_speed_sign * wheel_speed_turn;
					state4_d = 1;
				}
				else if (state4_d == 1)
				{
					//motor_pid[0].target = (-1) * left_speed_sign * wheel_speed_turn;
					//motor_pid[1].target = right_speed_sign * wheel_speed_turn;
					
					if (HAL_GetTick() - counter > medium_90_tick){ //turn left 90
						
						state4_d = 0;
						state = 1;
						
					}
				}
			}
			/* state 5 */
			else if (state == 5){ //turn right 90 degree
				if (state5_d == 0)
				{
					counter = HAL_GetTick();
					motor_pid[0].target = left_speed_sign * wheel_speed_turn;
					motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_turn;
					state5_d = 1;
				}
				else if (state5_d == 1)
				{
					//motor_pid[0].target = left_speed_sign * wheel_speed_turn;
					//motor_pid[1].target = (-1) * right_speed_sign * wheel_speed_turn;
					if (HAL_GetTick() - counter > medium_90_tick){ //turn right 90
						
						state5_d = 0;
						state = 1;
						
					}
				}
			}
			
		/****** End of State Machine *******/
			
			
		/****** Ultrasonics Detection *******/
		
		if(counter1 % 10 == 0){ /********* Ultrasonic detect every 100 ms ********/
			
			if(state == 3){ /*** front1_detect only ***/
				
				HAL_GPIO_WritePin(GPIOA, TRIGGER1_Pin, GPIO_PIN_RESET);
				usDelay(3);
				//*** START Ultrasonic measure routine ***//
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(GPIOA, TRIGGER1_Pin, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(GPIOA, TRIGGER1_Pin, GPIO_PIN_RESET);
				
				//2. Wait for ECHO pin rising edge
				while(HAL_GPIO_ReadPin(GPIOA, ECHO1_Pin) == GPIO_PIN_RESET);
				//3. Start measuring ECHO pulse width in usec
				numTicks = 0;
				while(HAL_GPIO_ReadPin(GPIOA, ECHO1_Pin) == GPIO_PIN_SET)
				{
					numTicks++;
					usDelay(2); //2.8usec
				};
		
				distance_front1 = (numTicks + 0.0f)*2.8*speedOfSound;
				numTicks = 0;
				
				if(distance_front1 < front_sonic_threshold){
					front1_detect = 1;
					HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_RESET);
				}
				else
				{
					front1_detect = 0;
					HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_SET);
				}
				
			
				/******** Front Sonic Left ********/
				HAL_GPIO_WritePin(TRIGGER4_GPIO_Port, TRIGGER4_Pin, GPIO_PIN_RESET);
				usDelay(3);
				//*** START Ultrasonic measure routine ***//
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(TRIGGER4_GPIO_Port, TRIGGER4_Pin, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(TRIGGER4_GPIO_Port, TRIGGER4_Pin, GPIO_PIN_RESET);
			
				//2. Wait for ECHO pin rising edge
				while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == GPIO_PIN_RESET);
		
				//3. Start measuring ECHO pulse width in usec
				numTicks = 0;
				while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == GPIO_PIN_SET)
				{
					numTicks++;
					usDelay(2); //2.8usec
				};
		
				distance_front2 = (numTicks + 0.0f)*2.8*speedOfSound;
				numTicks = 0;
				
				if(distance_front2 < front_sonic_threshold){
					
					 front2_detect = 1;
					 HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
				}
				
				else{
					front2_detect = 0;
					HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				}
				
				
				if(front1_detect || front2_detect){
					state = 3;

					//HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
				}
				else{
					state = 1; 
 
					//HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
				}
				
			}
			
			
			if (state == 1){ /******** Scan for enemy front, left, and right ********/
				
				/************** Front Sonic Right ****************/
				HAL_GPIO_WritePin(TRIGGER1_GPIO_Port, TRIGGER1_Pin, GPIO_PIN_RESET);
				usDelay(3);

				//*** START Ultrasonic measure routine ***//
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(TRIGGER1_GPIO_Port, TRIGGER1_Pin, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(TRIGGER1_GPIO_Port, TRIGGER1_Pin, GPIO_PIN_RESET);
			
				//2. Wait for ECHO pin rising edge
				while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == GPIO_PIN_RESET);
		
				//3. Start measuring ECHO pulse width in usec
				numTicks = 0;
				while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == GPIO_PIN_SET)
				{
					numTicks++;
					usDelay(2); //2.8usec
				};
		
				distance_front1 = (numTicks + 0.0f)*2.8*speedOfSound;
				numTicks = 0;
				
				if(distance_front1 < front_sonic_threshold){

					front1_detect = 1;
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				}
				else{

					front1_detect = 0;
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			
				}
				
				/*********** Front Sonic Left ***********/
				HAL_GPIO_WritePin(TRIGGER4_GPIO_Port, TRIGGER4_Pin, GPIO_PIN_RESET);
				usDelay(3);
				
				/*** START Ultrasonic measure routine ***/
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(TRIGGER4_GPIO_Port, TRIGGER4_Pin, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(TRIGGER4_GPIO_Port, TRIGGER4_Pin, GPIO_PIN_RESET);
			
				//2. Wait for ECHO pin rising edge
				while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == GPIO_PIN_RESET);
		
				//3. Start measuring ECHO pulse width in usec
				numTicks = 0;
				while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == GPIO_PIN_SET)
				{
					numTicks++;
					usDelay(2); //2.8usec
				};
				
		
				distance_front2 = (numTicks + 0.0f)*2.8*speedOfSound;
				numTicks = 0;
				
				if(distance_front2 < front_sonic_threshold){
					front2_detect = 1;
					HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
				}
				
				else{
					front2_detect = 0;
					HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
				}
				
				if(front1_detect || front2_detect){

					state = 3;
					//HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
				}
				else{
					state = 1; 
 
					//HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
				}
				
					
				/************* Left Sonic *****************/
				HAL_GPIO_WritePin(TRIGGER2_GPIO_Port, TRIGGER2_Pin, GPIO_PIN_RESET);
				usDelay(3);

				//*** START Ultrasonic measure routine ***//
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(TRIGGER2_GPIO_Port, TRIGGER2_Pin, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(TRIGGER2_GPIO_Port, TRIGGER2_Pin, GPIO_PIN_RESET);
			
				//2. Wait for ECHO pin rising edge
				while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_RESET);
		
				//3. Start measuring ECHO pulse width in usec
				numTicks = 0;
				while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_SET)
				{
					numTicks++;
					usDelay(2); //2.8usec
				};
		
				distance_left = (numTicks + 0.0f)*2.8*speedOfSound;
				numTicks = 0;
				
				if(distance_left < left_sonic_threshold){
					state = 4;
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

				}
				else{
					
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
				}
			
				
				/************ Right Sonic **************/
				HAL_GPIO_WritePin(TRIGGER3_GPIO_Port, TRIGGER3_Pin, GPIO_PIN_RESET);
				usDelay(3);

				//*** START Ultrasonic measure routine ***//
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(TRIGGER3_GPIO_Port, TRIGGER3_Pin, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(TRIGGER3_GPIO_Port, TRIGGER3_Pin, GPIO_PIN_RESET);
			
				//2. Wait for ECHO pin rising edge
				while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == GPIO_PIN_RESET);
		
				//3. Start measuring ECHO pulse width in usec
				numTicks = 0;
				while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == GPIO_PIN_SET)
				{
					numTicks++;
					usDelay(2); //2.8usec
				};
		
				distance_right = (numTicks + 0.0f)*2.8*speedOfSound;
				numTicks = 0;

				if(distance_right < right_sonic_threshold){
					state = 5;
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);

				}
				else{
					
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			
				}
				

			}
			
		}
		
		/////
		
		/****** End of Ultrasonic Detection ******/
				

		
    for(int i=0; i<4; i++)
    {	
      //motor_pid[i].target = set_spd; 																							
      motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //根据设定值进行PID计算。
    }
    set_moto_current(&hcan1, motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
                        motor_pid[1].output,
                        motor_pid[2].output,
                        motor_pid[3].output);
    
    HAL_Delay(10);      //PID控制频率100HZ

		/* USER CODE END WHILE */
		//usDelay(10);

		counter1++;
		
		
  }
	/* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */
//	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();

  }
	
	if (htim->Instance == TIM1) {

	
	}
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	TIM4->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	TIM4->EGR = 1; 			/*Re-initialises the timer*/
	TIM4->SR &= ~1; 		//Resets the flag
	TIM4->CR1 |= 1; 		//Enables the counter
	while((TIM4->SR&0x0001) != 1);
	TIM4->SR &= ~(0x0001);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
