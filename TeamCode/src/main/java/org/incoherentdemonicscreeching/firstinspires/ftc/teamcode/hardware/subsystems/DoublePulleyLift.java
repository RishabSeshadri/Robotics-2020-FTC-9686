package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;

public class DoublePulleyLift implements Subsystem {

    private YellowJacketEx m_left;
    private YellowJacketEx m_right;

    public DoublePulleyLift(YellowJacketEx left, YellowJacketEx right) {
        m_right = right;
        m_left = left;

        initialize();
    }

    @Override
    public void initialize() {
        m_right.setInverted(true);
        m_right.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        m_right.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BREAK);

        m_left.setInverted(false);
        m_left.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        m_left.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BREAK);
    }

    public void lift(double power) {
        m_right.pidWrite(power);
        m_left.pidWrite(power);
    }

    @Override
    public void reset() {
        m_right.resetController();
        m_right.resetEncoder();

        m_left.resetController();
        m_left.resetEncoder();
    }

    @Override
    public void loop() {
        m_right.set(0.5);
        m_left.set(0.5);
    }

    @Override
    public void stop() {
        m_right.stopMotor();
        m_left.stopMotor();
    }

    @Override
    public void disable() {
        m_right.disable();
        m_left.disable();
    }

}
