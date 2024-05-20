#!/usr/bin/env python
# coding=UTF-8


def _init():
    global _global_dict
    _global_dict = {}


def SetValue(name, value):
    _global_dict[name] = value


def GetValue(name):
    return _global_dict.get(name, 0)


def GetLen():
    return len(_global_dict)


class CanIDV2(object):
    # // control group: 0x1
    MOTION_COMMAND_ID = 0X111
    LIGHT_COMMAND_ID = 0X121
    BRAKING_COMMAND_ID = 0X131
    SET_MOTION_MODE_ID = 0X141

    # // state feedback group: 0x2
    SYSTEM_STATE_ID = 0X211
    MOTION_STATE_ID = 0X221
    LIGHT_STATE_ID = 0X231
    RC_STATE_ID = 0X241

    # //get catuator hight speed state
    ACTUATOR1_HS_STATE_ID = 0X251
    ACTUATOR2_HS_STATE_ID = 0X252
    ACTUATOR3_HS_STATE_ID = 0X253
    ACTUATOR4_HS_STATE_ID = 0X254
    ACTUATOR5_HS_STATE_ID = 0X255
    ACTUATOR6_HS_STATE_ID = 0X256
    ACTUATOR7_HS_STATE_ID = 0X257
    ACTUATOR8_HS_STATE_ID = 0X258

    # //get actuator low speed state
    ACTUATOR1_LS_STATE_ID = 0X261
    ACTUATOR2_LS_STATE_ID = 0X262
    ACTUATOR3_LS_STATE_ID = 0X263
    ACTUATOR4_LS_STATE_ID = 0X264
    ACTUATOR5_LS_STATE_ID = 0X265
    ACTUATOR6_LS_STATE_ID = 0X266
    ACTUATOR7_LS_STATE_ID = 0X267
    ACTUATOR8_LS_STATE_ID = 0X268

    MOTOR_ANGLE_INFO_ID = 0X271
    MOTOR_SPEED_INFO_ID = 0X281
    CURRENT_CTRL_MODE_ID = 0X291

    # // sensor data group: 0x3
    ODOMETRY_ID = 0X311
    IMU_ACCEL_ID = 0X321
    IMU_GYRO_ID = 0X322
    IMU_EULER_ID = 0X323
    SAFETY_BUMPER_ID = 0X331

    BMS_BASIC_ID = 0X361
    BMS_EXTENDED_ID = 0X362

    # // query/config group: 0x4
    VERSION_REQUEST_ID = 0X411
    VERSION_RESPONSE_ID = 0X41A
    CTRL_MODE_CONFIG_ID = 0X421
    STEER_NEUTRAL_REQUEST_ID = 0X431
    STEER_NEUTRAL_RESPONSE_ID = 0X43A
    STATE_RESET_CONFIG_ID = 0X441


class CanIDV1(object):
    # CAN: control group

    MOTION_COMMAND_ID = 0x130
    LIGHT_COMMAND_ID = 0x140
    VALUE_SET_COMMAND_ID = 0x210

    # CAN: state feedback group
    MOTION_STATE_ID = 0x131
    LIGHT_STATE_ID = 0x141
    SYSTEM_STATE_ID = 0x151

    VALUE_SET_STATE_ID = 0x211

    ACTUATOR1_STATE_ID = 0x200
    ACTUATOR2_STATE_ID = 0x201
    ACTUATOR3_STATE_ID = 0x202
    ACTUATOR4_STATE_ID = 0x203
