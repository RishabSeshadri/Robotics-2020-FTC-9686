package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.JSTEncoder;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.DoublePulleyLift;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.IntakeTwoWheel;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.OdometryGlobalCoordinatePosition;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.skystoneRobocop;


@Autonomous(name="First Autonomous")
public class BasicAuto extends LinearOpMode  {
    private ElapsedTime time = new ElapsedTime();
    private skystoneRobocop robot = new skystoneRobocop(hardwareMap);

    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    private Thread positionThread = new Thread(globalPositionUpdate);

    private YellowJacket435 m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDrive m_drive;

    private JSTEncoder m_encoderLeft, m_encoderRight, m_centralEncoder;

    private YellowJacket435 m_intakeLeft, m_intakeRight;
    private YellowJacketEx m_liftMotorL, m_liftMotorR;

    private IntakeTwoWheel m_intake;
    private DoublePulleyLift m_lift;

    private Servo m_foundationGrabberL, getM_foundationGrabberR;

    private RevIMU m_imu;
    private HolonomicOdometry m_odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setSafetyMode(Safety.SWIFT);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        robot.start();
        robot.reverseEncoders();
        //start of the autonomous. Aim: collect two blocks, put on foundation, set foundation into right place, and finally part into correct position

        while(opModeIsActive()){

            telemetry.addData("Robot Position", robot.getRobotPose());
            telemetry.addData("X Position", robot.returnXPosition());
            telemetry.addData("Y Position", robot.returnYPosition());
            telemetry.addData("Orientation (Degrees)", robot.orientation());

            telemetry.addData("Thread Active", robot.alive());
            telemetry.update();
        }
    }

    public void liftToPosition(double position, double power){
        m_frontLeft.setTargetPosition(position);
        m_frontRight.setTargetPosition(position);
        m_backLeft.setTargetPosition(position);
        m_backRight.setTargetPosition(position);
        m_frontLeft.pidWrite(power);
        m_frontRight.pidWrite(power);
        m_backLeft.pidWrite(power);
        m_backRight.pidWrite(power);
    }


}