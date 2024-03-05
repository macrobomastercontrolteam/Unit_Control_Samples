/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef enum
{
    // 8006 motor as hip
    // CAN_HIP1_TX_ID = 0x001,
    // CAN_HIP2_TX_ID = 0x002,
    // CAN_HIP3_TX_ID = 0x003,
    // CAN_HIP4_TX_ID = 0x004,

    // 6012 motor as hip
    // CAN_HIP1_RX_ID = 0x141,
    // CAN_HIP2_RX_ID = 0x142,
    // CAN_HIP3_RX_ID = 0x143,
    // CAN_HIP4_RX_ID = 0x144,

    // CAN_DRIVE1_PVT_TX_ID = 0x401,
    // CAN_DRIVE2_PVT_TX_ID = 0x402,
    // CAN_YAW_MOTOR_RX_ID = 0x205,
    // CAN_PIT_MOTOR_RX_ID = 0x206,
    // CAN_TRIGGER_MOTOR_RX_ID = 0x207,

    // engineer robotic arm
    CAN_JOINT_MOTOR_0_4310_ID = 0x001,
    CAN_JOINT_MOTOR_1_6012_ID = 0x142,
    CAN_JOINT_MOTOR_2_4010_ID = 0x143,
    CAN_JOINT_MOTOR_3_4310_ID = 0x004,
    CAN_JOINT_MOTOR_4_4310_ID = 0x005,
    CAN_JOINT_MOTOR_5_4310_ID = 0x006,
    CAN_JOINT_MOTOR_6_6020_RX_ID = 0x20B,

    CAN_GIMBAL_CONTROLLER_POSITION_RX_ID = 0x114,
	CAN_GIMBAL_CONTROLLER_ORIENTATION_RX_ID = 0x115,
	CAN_GIMBAL_CONTROLLER_INDIVIDUAL_MOTOR_1_RX_ID = 0x116,
	CAN_GIMBAL_CONTROLLER_INDIVIDUAL_MOTOR_2_RX_ID = 0x117,

} can_msg_id_e;

// Consecutive indices corresponding to individual can_msg_id_e
typedef enum
{
    // CHASSIS_ID_HIP_RF = 0, // right front
    // CHASSIS_ID_HIP_LF = 1, // left front
    // CHASSIS_ID_HIP_LB = 2, // left back
    // CHASSIS_ID_HIP_RB = 3, // right back
    // CHASSIS_ID_DRIVE_RIGHT = 4,
    // CHASSIS_ID_DRIVE_LEFT = 5,
    // CHASSIS_ID_YAW = 6,
    // CHASSIS_ID_PIT = 7,
    // CHASSIS_ID_TRIGGER = 8,

    // Engineer
    GIMBAL_ID_JOINT_0_4310 = 0,
    GIMBAL_ID_JOINT_1_6012 = 1,
    GIMBAL_ID_JOINT_2_4010 = 2,
    GIMBAL_ID_JOINT_3_4310 = 3,
    GIMBAL_ID_JOINT_4_4310 = 4,
    GIMBAL_ID_JOINT_5_4310 = 5,
    GIMBAL_ID_JOINT_6_6020 = 6,
    GIMBAL_ID_LAST,
} chassis_motor_ID_e;

typedef enum
{
    // CAN_GIMBAL_ALL_TX_ID = 0x1FF,

    // // 8006 motor as hip
    CAN_DAMIAO_RX_ID = 0x0FF,

    // // 6012 motor as hip
    // CAN_HIP_MOTOR_MULTICMD_TX_ID = 0x280,

    // /*******Chassis CAN IDs********/
    // CAN_DRIVE_MOTOR_PVT_RX_ID1 = 0x501,
    // CAN_DRIVE_MOTOR_PVT_RX_ID2 = 0x502,

    // CAN_DRIVE_MOTOR_SINGLECMD_TX_ID = 0x140,
    // CAN_DRIVE_MOTOR_MULTICMD_TX_ID = 0x280,
    // CAN_DRIVE_MOTOR_CMD_RX_ID1 = 0x241,
    // CAN_DRIVE_MOTOR_CMD_RX_ID2 = 0x242,

    // Enineer Arm
    CAN_JOINT_1_MOTOR_6012_RX_ID = 0x282,
    CAN_JOINT_2_MOTOR_4010_RX_ID = 0x283,

} can_other_msg_id_e;

typedef enum
{
    CAN_6012_TORQUE_RX_ID = 0xA1,
    CAN_9015_MULTIANGLE_MSG_ID = 0x92,
    CAN_9015_SET_CURRENT_ZERO_POINT_MSG_ID = 0x64,
} can_msg_type_e;

typedef enum
{
    DM_8006 = 0,
    MA_9015 = 1,
    DM_4310 = 2,
    MG_6012 = 3,
    MG_4010 = 4,

    LAST_MOTOR_TYPE = MG_4010 + 1,
} motor_type_e;

// rm motor data
typedef union
{
    int8_t temperature;
    struct
    {
        uint16_t ecd;
        int16_t speed_rpm;
        int16_t given_current;
        int16_t last_ecd;
    };
    struct
    {
        fp32 output_angle; // rad
        fp32 input_angle;  // rad
        fp32 velocity;     // rad/s
        fp32 torque;       // Nm
        fp32 power;        //
    };
} motor_measure_t;

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
// uint8_t get_motor_array_index(can_msg_id_e _SINGLE_CAN_ID);
// uint8_t hip_motor_set_torque(float RF_torq, float LF_torq, float LB_torq, float RB_torq);
uint8_t arm_joints_set_position(float *joint_target_pos_ptr);

extern motor_measure_t motor_measure[GIMBAL_ID_LAST];

void enable_all_motor_control(uint8_t _enable);

#ifdef __cplusplus
}
#endif

#endif /* CAN_RECEIVE_H */
