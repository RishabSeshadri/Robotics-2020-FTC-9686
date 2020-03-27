package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;

public class IntakeTwoWheel implements Subsystem {

    private YellowJacket435 m_left, m_right;

    public IntakeTwoWheel(YellowJacket435 left, YellowJacket435 right) {
        m_left = left;
        m_right = right;

        initialize();
    }

    @Override
    public void initialize() {
        m_left.setInverted(false);
        m_right.setInverted(true);
    }

    @Override
    public void reset() {
        m_left.resetPIDOutput();
        m_right.resetPIDOutput();
    }

    /**
     * Binds to a trigger
     */
    public void intake() {
        m_left.set(0.83);
        m_right.set(0.83);
    }

    /**
     * Binds to a trigger
     */
    public void outtake() {
        m_left.set(-0.67);
        m_right.set(-0.67);
    }

    /**
     * Used for autonomous
     */
    @Override
    public void loop() {
        m_left.pidWrite(0.75);
        m_right.pidWrite(0.75);
    }

    @Override
    public void stop() {
        m_left.stopMotor();
        m_right.stopMotor();
        reset();
    }

    @Override
    public void disable() {
        stop();
        m_left.disable();
        m_right.disable();
    }

}
