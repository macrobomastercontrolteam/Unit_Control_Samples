# Unit_Control_Samples
## Directories
- GM6020_swing:
  - Written by Yuntian Wang in 2022, based on DJI's demo code, "RoboMaster GM6020 Brushless DC Motor Demo", on https://www.robomaster.com/en-US/products/components/general/gm6020#downloads
  - Type A Dev Board controls a GM6020 BLDC motor
  - Swings GM6020 around the point at the moment when reset button is pressed. Two simple finite state machines are used for demo purpose only. Normally angle PID control is used for this application.
  - RPM and left-right angle limits are fixed and defined by macros in the code.
- Sumobot_2020:
  - Written by Jinge Li for Sumobot competition in 2020 (not sure).
  - Type A Dev Board controls four M3508 BLDC motors
  - Timer configs
    - TIM1: unused
    - TIM4: used to generate usDelay; was not included in stm32cubemx config but added manually
    - TIM6: can be changed to systick
- JointMotorTests:
  - Written in 2024 as a collection of joint motor controlling helper functions
  - Type C Dev Board
  - Motors tested
   - DM-J8006-2EC by DaMiao(达妙), http://www.dmbot.cn/
   - MG6012E-i36 by KTech(瓴控科技), https://vi.aliexpress.com/item/1005005528411352.html
   - RMD-L-9015-35T by MyActuator(脉塔), https://www.myactuator.com/product-page/rmd-l-9015