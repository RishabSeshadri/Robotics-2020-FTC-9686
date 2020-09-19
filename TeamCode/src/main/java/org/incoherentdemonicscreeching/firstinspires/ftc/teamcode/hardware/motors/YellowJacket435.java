package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class YellowJacket435 implements Motor {

    public static final double RPM = 435.0;                 // rev per minute
    public static final double CPR = 383.6;                 // ticks per rev
    public static final double MAX_VEL = CPR * RPM / 60.0;  // ticks per second

    /**
     * These are calculated for velocity PID
     */
    public static final double kF = 32767.0 / MAX_VEL;
    public static final double kP = 0.1 * kF;
    public static final double kI = 0.1 * kP;
    public static final double kD = 0;

    private DcMotor m_motor;
    private PIDFController controller =
            new PIDFController(new double[]{kP, kI, kD, kF});

    public YellowJacket435(HardwareMap hMap, String name, boolean isInverted) {
        m_motor = hMap.get(DcMotor.class, name);
        setInverted(isInverted);
    }

    public YellowJacket435(HardwareMap hMap, String name) {
        this(hMap, name, false);
    }

    @Override
    public void set(double speed) {
        m_motor.setPower(speed);
    }

    @Override
    public double get() {
        return m_motor.getPower();
    }

    public double getPosition() {
        return m_motor.getCurrentPosition();
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_motor.setDirection(isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    public void resetPIDOutput() {
        controller.reset();
    }

    @Override
    public boolean getInverted() {
        return m_motor.getDirection() == DcMotor.Direction.REVERSE;
    }

    @Override
    public void disable() {
        m_motor.close();
    }

    @Override
    public String getDeviceType() {
        return "Yellow Jacket, 435 rpm";
    }

    /**
     * Setting the speed in terms of rotations per minute (rpm)
     *
     * @param speed the speed in rpm
     */
    public void setSpeed(double speed) {
        set(speed / RPM);
    }

    /**
     * @param output percentage of output speed
     */
    @Override
    public void pidWrite(double output) {
        set(get() + controller.calculate(output, get()));
    }

    @Override
    public void stopMotor() {
        set(0);
    }

    public void setTargetPosition(double position) {
    }
}
