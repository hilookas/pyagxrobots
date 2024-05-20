import pyagxrobots.UGVConfigMsg as UGVBaseMsg


class MotionCommandMessage(object):
    def GetLinearVelocity(self):
        return float(UGVBaseMsg.GetValue('LinearVelocity'))

    def GetAngularVelocity(self):
        # // only valid for differential drivering
        return float(UGVBaseMsg.GetValue('AngularVelocity'))

    def GetLateralVelocity(self):
        return float(UGVBaseMsg.GetValue('LateralVelocity'))

    def GetSteeringAngle(self):
        # // only valid for ackermann steering
        return float(UGVBaseMsg.GetValue('SteeringAngle'))


class LightCommandMessage(object):
    def GetLightCmdCtrl(self):
        return UGVBaseMsg.GetValue('LightCmdCtrl')

    def GetFrontMode(self):
        return UGVBaseMsg.GetValue('FrontMode')

    def GetFrontCustom(self):
        return UGVBaseMsg.GetValue('FrontCustom')

    def GetRearMode(self):
        return UGVBaseMsg.GetValue('RearMode')

    def GetRearCustom(self):
        return UGVBaseMsg.GetValue('RearCustom')


class SystemStateMessage(object):
    def GetVehicleState(self):
        return UGVBaseMsg.GetValue('VehicleState')

    def GetControlMode(self):
        return UGVBaseMsg.GetValue('ControlMode')

    def GetBatteryVoltage(self):
        return float(UGVBaseMsg.GetValue('BatteryVoltage'))

    def GetErrorCode(self):
        return UGVBaseMsg.GetValue('ErrorCode')


class RcStateMessage(object):
    def GetVarA(self):
        return UGVBaseMsg.GetValue('VarA')

    def GetSws(self):
        return UGVBaseMsg.GetValue('Sws')

    def GetStickRightV(self):
        return UGVBaseMsg.GetValue('StickRightV')

    def GetStickRightH(self):
        return UGVBaseMsg.GetValue('StickRightH')

    def GetStickLeftV(self):
        return UGVBaseMsg.GetValue('StickLeftV')

    def GetStickLeftH(self):
        return UGVBaseMsg.GetValue('StickLeftH')


class OdometryMessage(object):
    def GetLeftWheel(self):
        return float(UGVBaseMsg.GetValue('LeftWheel'))

    def GetRightWheel(self):
        return float(UGVBaseMsg.GetValue('RightWheel'))


class ActuatorStateMessageV1(object):
    def __init__(self, motro_id=0):
        self.motro_id = motro_id

    def current(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Current1')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Current2')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Current3')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Current4')

    def rpm(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Rpm1')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Rpm2')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Rpm3')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Rpm4')

    def driver_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Driver1Temp')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Driver2Temp')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Driver3Temp')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Driver4Temp')

    def motor_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Motor1Temp')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Motor2Temp')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Motor3Temp')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Motor4Temp')


class ActuatorStateMessageV2(object):
    def __init__(self, motro_id=0):
        self.motro_id = motro_id

    def rpm(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Rpm1')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Rpm2')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Rpm3')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Rpm4')

    def current(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Current1')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Current2')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Current3')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Current4')

    def pulse_count(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('PulseCount1')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('PulseCount2')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('PulseCount3')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('PulseCount4')

    def driver_voltage(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Driver1Voltage')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Driver2Voltage')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Driver3Voltage')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Driver4Voltage')

    def driver_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Driver1Temp')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Driver2Temp')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Driver3Temp')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Driver4Temp')

    def motor_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Motor1Temp')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Motor2Temp')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Motor3Temp')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Motor4Temp')

    def driver_state(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetValue('Driver1State')
        elif self.motro_id == 2:
            return UGVBaseMsg.GetValue('Driver2State')
        elif self.motro_id == 3:
            return UGVBaseMsg.GetValue('Driver3State')
        elif self.motro_id == 4:
            return UGVBaseMsg.GetValue('Driver4State')


class GetRobotStae(MotionCommandMessage,
                   LightCommandMessage,
                   SystemStateMessage,
                   RcStateMessage,
                   OdometryMessage):
    pass
