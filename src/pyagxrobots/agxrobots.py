import can
import ctypes
import pyagxrobots.UGVConfigMsg as UGVBaseMsg
import time

class UGV:
    UGVBaseMsg._init()

    def __init__(self, bustype="socketcan", channel="can0", bitrate=500000):
        UGVBaseMsg._init()

        self.bus = can.Bus(bustype=bustype, channel=channel, bitrate=bitrate)

        # Get base version
        while True:
            msg = self.bus.recv(0.01)
            if msg:
                if msg.arbitration_id == 0x151:
                    self.base_version = 1
                    break
                elif msg.arbitration_id == 0x241:
                    self.base_version = 2
                    break
            time.sleep(0.01)

        # Control the chassis in Can control mode
        # you need to dial the remote control SWB to the top
        if self.base_version == 2:
            self.SendMessage(UGVBaseMsg.CanIDV2.CTRL_MODE_CONFIG_ID, [0x01])

        self.notifier = can.Notifier(self.bus, [ self.ListenMessage, ])

    def stop(self):
        self.notifier.stop()
        self.bus.shutdown()

    def SendMessage(self, id, data):
        try:
            self.bus.send(can.Message(arbitration_id=id, is_extended_id=False, data=data))
        except can.CanError:
            print("Message NOT sent")

    def SetMotionCommand(self,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0,
        lateral_vel: float = 0.0,
        steering_angle: float = 0.0
    ):
        if self.base_version == 1:
            if abs(angular_vel) > abs(steering_angle):
                angular = angular_vel
            else:
                angular = steering_angle
            self.SendMessage(UGVBaseMsg.CanIDV1.MOTION_COMMAND_ID, [
                0x01,
                0,
                (int(linear_vel)& 0xff),
                (int(angular) & 0xff),
                (int(lateral_vel) & 0xff),
                0,
                0,
                0,
            ])
        elif self.base_version == 2:
            linear_cmd = linear_vel * 1000
            angular_cmd = angular_vel * 1000
            lateral_cmd = lateral_vel * 1000
            steering_cmd = steering_angle * 1000
            self.SendMessage(UGVBaseMsg.CanIDV2.MOTION_COMMAND_ID, [
                int(linear_cmd) >> 8 & 0xff,
                (int(linear_cmd) & 0x00ff),
                int(angular_cmd) >> 8 & 0xff,
                (int(angular_cmd) & 0x00ff),
                int(lateral_cmd) >> 8 & 0xff,
                (int(lateral_cmd) & 0x00ff),
                int(steering_cmd) >> 8 & 0xff,
                (int(steering_cmd) & 0x00ff)
            ])

    def SetLightCommand(self,
        front_mode: int = 0,
        front_custom_value: int = 0,
        rear_mode: int = 0,
        rear_custom_value: int = 0
    ):
        if self.base_version == 1:
            self.SendMessage(UGVBaseMsg.CanIDV1.LIGHT_COMMAND_ID, [
                0x01,
                front_mode,
                front_custom_value,
                rear_mode,
                rear_custom_value,
                0,
                0,
                0
            ])
        elif self.base_version == 2:
            self.SendMessage(UGVBaseMsg.CanIDV2.LIGHT_COMMAND_ID, [
                0x01,
                front_mode,
                front_custom_value,
                rear_mode,
                rear_custom_value,
                0,
                0,
                0
            ])

    def ProcessMessageV1(self, msg):
        if (msg.arbitration_id == UGVBaseMsg.CanIDV1.SYSTEM_STATE_ID):
            vehicle_state = int(msg.data[0])
            UGVBaseMsg.SetValue('VehicleState', vehicle_state)
            control_mode = msg.data[1]
            UGVBaseMsg.SetValue('ControlMode', control_mode)
            battery_voltage = float((msg.data[2] & 0xff) << 8
                                    | msg.data[3]) / 10
            UGVBaseMsg.SetValue('BatteryVoltage', battery_voltage)
            error_code = (msg.data[4] << 8) | msg.data[5]
            UGVBaseMsg.SetValue('ErrorCode', error_code)

            # print(
            #     'vehicle_state:%s control_mode:%s battery_voltage:%s error_code:%s  count_num:%s'
            #     % (vehicle_state, control_mode, battery_voltage, error_code,
            #        count_num))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV1.MOTION_STATE_ID):
            linear_velocity_get = ctypes.c_int16((msg.data[0] & 0xff) << 8
                                                 | msg.data[1])

            linear_velocity = float(linear_velocity_get.value / 1000)
            # print(int(linear_velocity))
            UGVBaseMsg.SetValue('LinearVelocity', linear_velocity)
            angular_velocity_get = ctypes.c_int16((msg.data[2] & 0xff) << 8
                                                  | msg.data[3])
            angular_velocity = float(angular_velocity_get.value / 1000)
            UGVBaseMsg.SetValue('AngularVelocity', angular_velocity)
            lateral_velocity_get = ctypes.c_int16((msg.data[4] & 0xff) << 8
                                                  | msg.data[5])
            lateral_velocity = float(lateral_velocity_get.value / 1000)
            UGVBaseMsg.SetValue('LateralVelocity', lateral_velocity)

            # print(msg)

            # print(msg)
            # print(
            #     'linear_velocity:%s angular_velocity:%s lateral_velocity:%s steering_angle:%s '
            #     % (linear_velocity, angular_velocity, lateral_velocity,
            #        steering_angle))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV1.LIGHT_STATE_ID):
            light_cmd_ctrl = (msg.data[0])
            UGVBaseMsg.SetValue('LightCmdCtrl', light_cmd_ctrl)
            front_mode = (msg.data[1])
            UGVBaseMsg.SetValue('FrontMode', front_mode)
            front_custom = (msg.data[2])
            UGVBaseMsg.SetValue('FrontCustom', front_custom)
            rear_mode = (msg.data[3])
            UGVBaseMsg.SetValue('RearMode', rear_mode)
            rear_custom = (msg.data[4])
            UGVBaseMsg.SetValue('RearCustom', rear_custom)

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV1.VALUE_SET_STATE_ID):
            if (msg.data[0] == 0xaa):
                base_neutral = True
            else:
                base_neutral = False
            UGVBaseMsg.SetValue('BaseNeutral', base_neutral)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR1_STATE_ID):
            current1 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetValue('Current1', current1)
            rpm1 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetValue('Rpm1', rpm1)
            driver1_temp = msg.date[4]
            UGVBaseMsg.SetValue('Driver1State', driver1_temp)
            motor1_temp = msg.data[5]
            UGVBaseMsg.SetValue('Motor1Temp', motor1_temp)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR2_STATE_ID):
            current2 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetValue('Current2', current2)
            rpm2 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetValue('Rpm2', rpm2)
            driver2_temp = msg.date[4]
            UGVBaseMsg.SetValue('Driver2State', driver2_temp)
            motor2_temp = msg.data[5]
            UGVBaseMsg.SetValue('Motor2Temp', motor2_temp)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR3_STATE_ID):
            current3 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetValue('Current3', current3)
            rpm3 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetValue('Rpm3', rpm3)
            driver3_temp = msg.date[4]
            UGVBaseMsg.SetValue('Driver3State', driver3_temp)
            motor3_temp = msg.data[5]
            UGVBaseMsg.SetValue('Motor3Temp', motor3_temp)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR4_STATE_ID):
            current4 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetValue('Current4', current4)
            rpm4 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetValue('Rpm4', rpm4)
            driver4_temp = msg.date[4]
            UGVBaseMsg.SetValue('Driver4State', driver4_temp)
            motor4_temp = msg.data[5]
            UGVBaseMsg.SetValue('Motor4Temp', motor4_temp)

    def ProcessMessageV2(self, msg):
        """
        Process Can bus data according to ID
        """

        msg = msg
        if (msg.arbitration_id == UGVBaseMsg.CanIDV2.SYSTEM_STATE_ID):
            vehicle_state = int(msg.data[0])
            UGVBaseMsg.SetValue('VehicleState', vehicle_state)
            control_mode = msg.data[1]
            UGVBaseMsg.SetValue('ControlMode', control_mode)
            battery_voltage = float((msg.data[2] & 0xff) << 8
                                    | msg.data[3]) / 10
            UGVBaseMsg.SetValue('BatteryVoltage', battery_voltage)
            error_code = msg.data[5]
            UGVBaseMsg.SetValue('ErrorCode', error_code)
            count_num = msg.data[7]

            # print(
            #     'vehicle_state:%s control_mode:%s battery_voltage:%s error_code:%s  count_num:%s'
            #     % (vehicle_state, control_mode, battery_voltage, error_code,
            #        count_num))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.MOTION_STATE_ID):
            linear_velocity_get = ctypes.c_int16((msg.data[0] & 0xff) << 8
                                                 | msg.data[1])

            linear_velocity = float(linear_velocity_get.value / 1000)
            # print(int(linear_velocity))
            UGVBaseMsg.SetValue('LinearVelocity', linear_velocity)
            angular_velocity_get = ctypes.c_int16((msg.data[2] & 0xff) << 8
                                                  | msg.data[3])
            angular_velocity = float(angular_velocity_get.value / 1000)
            UGVBaseMsg.SetValue('AngularVelocity', angular_velocity)
            lateral_velocity_get = ctypes.c_int16((msg.data[4] & 0xff) << 8
                                                  | msg.data[5])
            lateral_velocity = float(lateral_velocity_get.value / 1000)
            UGVBaseMsg.SetValue('LateralVelocity', lateral_velocity)
            steering_angle_get = ctypes.c_int16((msg.data[6] & 0xff) << 8
                                                | msg.data[7])
            steering_angle = float(steering_angle_get.value / 1000)
            UGVBaseMsg.SetValue('SteeringAngle', steering_angle)
            # print(msg)

            # print(msg)
            # print(
            #     'linear_velocity:%s angular_velocity:%s lateral_velocity:%s steering_angle:%s '
            #     % (linear_velocity, angular_velocity, lateral_velocity,
            #        steering_angle))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR1_HS_STATE_ID):
            rpm1 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetValue('Rpm1', rpm1)
            current1 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetValue('Current1', current1)
            pulse_count1 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetValue('PulseCount1', pulse_count1)
            # print('rpm1:%s current1:%s pulse_count1:%s ' %
            #       (rpm1, current1, pulse_count1))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR2_HS_STATE_ID):
            rpm2 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetValue('Rpm2', rpm2)
            current2 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetValue('Current2', current2)
            pulse_count2 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetValue('PulseCount2', pulse_count2)
            # print('rpm2:%s current2:%s pulse_count2:%s ' %
            #       (rpm2, current2, pulse_count2))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR3_HS_STATE_ID):
            rpm3 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetValue('Rpm3', rpm3)
            current3 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetValue('Current3', current3)
            pulse_count3 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetValue('PulseCount3', pulse_count3)
            # print('rpm3:%s current3:%s pulse_count3:%s ' %
            #       (rpm3, current3, pulse_count3))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR4_HS_STATE_ID):
            rpm4 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetValue('Rpm4', rpm4)
            current4 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetValue('Current4', current4)
            pulse_count4 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetValue('PulseCount4', pulse_count4)

            # print('rpm4:%s current4:%s pulse_count4:%s ' %
            #       (rpm4, current4, pulse_count4))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR1_LS_STATE_ID):
            driver1_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetValue('Driver1Voltage', driver1_voltage)
            driver1_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetValue('Driver1Temp', driver1_temp)
            motor1_temp = msg.data[4]
            UGVBaseMsg.SetValue('Motor1Temp', motor1_temp)
            driver1_state = msg.data[5]
            UGVBaseMsg.SetValue('Driver1State', driver1_state)
            # print(
            #     'driver_voltage1:%s driver_temp1:%s motor_temp%s driver_state1:%s '
            #     % (driver1_voltage, driver1_temp, motor1_temp, driver1_state))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR2_LS_STATE_ID):
            driver2_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetValue('Driver2Voltage', driver2_voltage)
            driver2_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetValue('Driver2Temp', driver2_temp)
            motor2_temp = msg.data[4]
            UGVBaseMsg.SetValue('Motor2Temp', motor2_temp)
            driver2_state = msg.data[5]
            UGVBaseMsg.SetValue('Driver2State', driver2_state)
            # print(
            #     'driver_voltage2:%s driver_temp2:%s  motor_temp2:%s driver_state2:%s '
            #     % (driver2_voltage, driver2_temp, motor2_temp, driver2_state))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR3_LS_STATE_ID):
            driver3_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetValue('Driver3Voltage', driver3_voltage)
            driver3_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetValue('Driver3Temp', driver3_temp)
            motor3_temp = msg.data[4]
            UGVBaseMsg.SetValue('Motor3Temp', motor3_temp)
            driver3_state = msg.data[5]
            UGVBaseMsg.SetValue('Driver3State', driver3_state)
            # print(
            #     'driver_voltage3:%s driver_temp3:%s  motor_temp3:%s driver_state3:%s '
            #     % (driver3_voltage, driver3_temp, motor3_temp, driver3_state))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR4_LS_STATE_ID):
            driver4_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetValue('Driver4Voltage', driver4_voltage)
            driver4_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetValue('Driver4Temp', driver4_temp)
            motor4_temp = msg.data[4]
            UGVBaseMsg.SetValue('Motor4Temp', motor4_temp)
            driver4_state = msg.data[5]
            UGVBaseMsg.SetValue('Driver4State', driver4_state)
            # print(
            #     'driver_voltage4:%s driver_temp4:%s  motor_temp:%s driver_state4:%s '
            #     % (driver4_voltage, driver4_temp, motor4_temp, driver4_state))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.LIGHT_STATE_ID):
            light_cmd_ctrl = (msg.data[0])
            UGVBaseMsg.SetValue('LightCmdCtrl', light_cmd_ctrl)
            front_mode = (msg.data[1])
            UGVBaseMsg.SetValue('FrontMode', front_mode)
            front_custom = (msg.data[2])
            UGVBaseMsg.SetValue('FrontCustom', front_custom)
            rear_mode = (msg.data[3])
            UGVBaseMsg.SetValue('RearMode', rear_mode)
            rear_custom = (msg.data[4])
            UGVBaseMsg.SetValue('RearCustom', rear_custom)

            # print(
            #     'enable_cmd_ctrl:%s front_mode:%s  front_custom:%s rear_mode:%s  rear_custom:%s'
            #     % (light_cmd_ctrl, front_mode, front_custom, rear_mode,
            #        rear_custom))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.VERSION_RESPONSE_ID):
            control_hardware_version = int((msg.data[0] & 0xff) << 8
                                           | msg.data[1])
            UGVBaseMsg.SetValue('ControlHardwareVersion', control_hardware_version)
            actuaror_hardware_version = int((msg.data[2] & 0xff) << 8
                                            | msg.data[3])
            UGVBaseMsg.SetValue('ActuarorHardwareVersion', actuaror_hardware_version)
            control_software_version = int((msg.data[4] & 0xff) << 8
                                           | msg.data[5])
            UGVBaseMsg.SetValue('ControlSoftwareVersion', control_software_version)
            actuaror_software_version = int((msg.data[6] & 0xff) << 8
                                            | msg.data[7])
            UGVBaseMsg.SetValue('ActuarorSoftwareVersion', actuaror_software_version)
            # print(
            #     'control_hardware_version: %s actuaror_hardware_version: %s' %
            #     (control_hardware_version, actuaror_hardware_version))
            # print('control_software_version:%s actuaror_software_version:%s ' %
            #       (control_software_version, actuaror_software_version))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ODOMETRY_ID):
            left_wheel_get = ctypes.c_int((msg.data[0] & 0xff) << 24
                                          | (msg.data[1] & 0xff) << 16
                                          | (msg.data[2] & 0xff) << 8
                                          | msg.data[3])
            left_wheel = left_wheel_get.value
            UGVBaseMsg.SetValue('LeftWheel', left_wheel)
            right_wheel_get = ctypes.c_int((msg.data[4] & 0xff) << 24
                                           | (msg.data[5] & 0xff) << 16
                                           | (msg.data[6] & 0xff) << 8
                                           | msg.data[7])
            # print('left_wheel: %s right_wheel: %s ' %
            #       (left_wheel, right_wheel))
            right_wheel = right_wheel_get.value
            UGVBaseMsg.SetValue('RightWheel', right_wheel)
            # print(msg)
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.RC_STATE_ID):
            sws = msg.data[0]
            UGVBaseMsg.SetValue('Sws', sws)
            stick_right_v = msg.data[1]
            UGVBaseMsg.SetValue('StickRightV', stick_right_v)
            stick_right_h = msg.data[2]
            UGVBaseMsg.SetValue('StickRightH', stick_right_h)
            stick_left_v = msg.data[3]
            UGVBaseMsg.SetValue('StickLeftV', stick_left_v)
            stick_left_h = msg.data[4]
            UGVBaseMsg.SetValue('StickLeftH', stick_left_h)
            var_a = msg.data[5]
            UGVBaseMsg.SetValue('VarA', var_a)

    def ListenMessage(self, msg):
        if self.base_version == 1:
            self.ProcessMessageV1(msg)
        elif self.base_version == 2:
            self.ProcessMessageV2(msg)
