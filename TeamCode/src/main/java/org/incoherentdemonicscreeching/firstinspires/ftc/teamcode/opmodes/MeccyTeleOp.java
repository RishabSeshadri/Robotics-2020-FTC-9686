package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.DoublePulleyLift;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.IntakeTwoWheel;

@TeleOp(name="Meccy TeleOp")
public class MeccyTeleOp extends OpMode {

    private YellowJacket435 m_frontLeft, m_frontRight, m_backLeft, m_backRight;

    private YellowJacket435 m_intakeLeft, m_intakeRight;

    private YellowJacket435 m_rightL, m_leftL;
    private YellowJacketEx m_leftLift, m_rightLift;

    private Servo m_grabby;

    private IntakeTwoWheel m_intake;
    private DoublePulleyLift m_lift;
    private MecanumDrive m_drive;

    private RevIMU m_imu;

    @Override
    public void init() {
        m_frontLeft = new YellowJacket435(hardwareMap, "fl");
        m_frontRight = new YellowJacket435(hardwareMap, "fr");
        m_backLeft = new YellowJacket435(hardwareMap, "bl");
        m_backRight = new YellowJacket435(hardwareMap, "br");

        m_leftL = new YellowJacket435(hardwareMap, "lL");
        m_rightL = new YellowJacket435(hardwareMap, "rL");

        m_leftLift = new YellowJacketEx(m_leftL);
        m_rightLift = new YellowJacketEx(m_rightL);

        m_intakeLeft = new YellowJacket435(hardwareMap, "iL");
        m_intakeRight = new YellowJacket435(hardwareMap, "iR");

        m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
        m_intake = new IntakeTwoWheel(m_intakeLeft, m_intakeRight);
        m_lift = new DoublePulleyLift(m_leftLift, m_rightLift);

        m_imu = new RevIMU(hardwareMap);

        m_grabby = hardwareMap.servo.get("grabby");
    }

    @Override
    public void loop() {
        m_drive.driveFieldCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                Math.toRadians(m_imu.getHeading())
        );

        if(gamepad1.right_trigger >= 0.15){
            m_intake.intake();
        } else if (gamepad1.left_trigger >= 0.15){
            m_intake.outtake();
        } else {
            m_intake.stop();
        }

        if(gamepad1.dpad_up){
            m_lift.lift(0.8);
        } else if(gamepad1.dpad_down){
            m_lift.lift(-0.8);
        } else {
            m_lift.stop();
        }

        //RANDOM POSITIONS BC I DONT HAVE AN ACTUAL SERVO IN QUARANTINEEEEEEEEEEEEEEEE

        if(gamepad1.y){
            m_grabby.setPosition(1);
        } else if (gamepad1.a){
            m_grabby.setPosition(0);
        } else {
            m_grabby.setPosition(0.5);
        }
    }

}
