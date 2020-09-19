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


        /*
        This part of the program first intakes a block then goes back to the first position.
         */
        robot.drive(10 , 24, 1, 90, 0.1, 1);
        robot.intake();
        time.wait(2000);
        robot.stopIntake();
        robot.drive(0, 0, 1, 0, 0.1,1);
        /*
        The second part of this autonomous would go to the foundation and put the block into the foundation.
         */
        robot.drive(-100, 0, 1, 180, 0.1 , 1);
        robot.drive(-100, 48, 1, 0, 0.1, 1);
        robot.drive(-100, 48, 1, 180, 0.1, 1);
        robot.liftToPosition(-3 , 0.25);
        robot.liftToPosition(3, 0.25);
        /*
        This next part would put the foundation into the zone and then travel back to the original coordinate
         */
        robot.drive(-100, 24, 0.5, 180, 0.1, 1);
        robot.drive(-100, -24, 0.5, 90, 0.1, 1);
        robot.drive(-110, -24, 0.5, 90, 0.1 ,1);

        robot.drive(0 , 0 , 1, 0, 0.1, 1);
        //This puts the robot back into the same position it was earlier
        robot.drive(-20, 0, 1, 0, 0.1,1);
        /*Second part of autonomous
        This first section of the second part of the autonomous intakes a block and makes the robot go back to the position.
         */
        robot.drive(20, 24 , 1, 90, 0.1, 1);
        robot.intake();
        time.wait(2000);
        robot.stopIntake();
        robot.drive(0, 0, 1, 0, 0.1, 1);
        robot.drive(0,0, 1,90, 0.1 , 1);
        /*
        This section of the program makes the robot drive all the way to the foundation and puts a block in the foundation. After that the autonomous would end after the robot goes under the bridge.
         */
        robot.drive(-110, 0, 1, 90, 0.1, 1);
        robot.liftToPosition(-3, 0.25);
        robot.liftToPosition(9, 0.25);
        robot.drive(-50, 0, 1, 90, 0.1, 1);




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