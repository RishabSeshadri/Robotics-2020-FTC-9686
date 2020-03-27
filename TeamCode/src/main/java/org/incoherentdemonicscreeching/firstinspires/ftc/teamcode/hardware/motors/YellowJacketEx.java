package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class YellowJacketEx extends MotorEx {

    public YellowJacketEx(YellowJacket435 motor) {
        super(motor, YellowJacket435.CPR);
    }

    @Override
    public double getCurrentPosition() {
        return ((YellowJacket435)motor).getPosition();
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.disable();
    }

    @Override
    public String getDeviceType() {
        return motor.getDeviceType();
    }

    /**
     * @param output percentage of output speed
     */
    @Override
    public void pidWrite(double output) {
        motor.pidWrite(output);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

}
