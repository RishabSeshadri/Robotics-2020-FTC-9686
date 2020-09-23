package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.JSTEncoder;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.DoublePulleyLift;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.IntakeTwoWheel;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.subsystems.OdometryGlobalCoordinatePosition;

public class skystoneRobocop extends Robot {

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

    private final double ROBOT_COUNTS_PER_INCH = 8192 / (75/25.4 * Math.PI);

    public skystoneRobocop(HardwareMap hardwareMap) {
        m_frontLeft = new YellowJacket435(hardwareMap, "fl");
        m_frontRight = new YellowJacket435(hardwareMap, "fr");
        m_backLeft = new YellowJacket435(hardwareMap, "bl");
        m_backRight = new YellowJacket435(hardwareMap, "br");

        m_intakeLeft = new YellowJacket435(hardwareMap, "il");
        m_intakeRight = new YellowJacket435(hardwareMap, "ir");

        m_liftMotorL = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "liftl")
        );
        m_liftMotorR = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "liftl")
        );

        m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

        m_lift = new DoublePulleyLift(m_liftMotorL, m_liftMotorR);
        m_intake = new IntakeTwoWheel(m_intakeLeft, m_intakeRight);

        m_imu = new RevIMU(hardwareMap);
        m_odometry = new HolonomicOdometry(16.77);

        m_encoderLeft = new JSTEncoder(hardwareMap, "encoderLeft");
        m_encoderRight = new JSTEncoder(hardwareMap, "encoderRight");
        m_centralEncoder = new JSTEncoder(hardwareMap, "encoderCenter");

        m_encoderLeft.setDistancePerPulse(4.0 * Math.PI / 8092);
        m_encoderRight.setDistancePerPulse(4.0 * Math.PI / 8092);
        m_centralEncoder.setDistancePerPulse(4.0 * Math.PI / 8092);
    }

    public void drive(double x, double y, double turn) {
        if (m_safety == Safety.SWIFT) {
            m_drive.driveFieldCentric(
                    x, y, turn, Math.toRadians(m_imu.getHeading())
            );
        } else {
            m_drive.driveFieldCentric(
                    x, y, turn, Math.toRadians(m_imu.getHeading()), true
            );
        }
    }

    public void intake() {
        m_intake.intake();
    }

    public void outtake() {
        m_intake.outtake();
    }

    public void stopIntake() {
        m_intake.stop();
    }

    public void actuateLift(double speed) {
        m_lift.lift(speed);
    }

    public void disable() {
        m_drive.stopMotor();
        m_lift.disable();
        m_intake.disable();
    }

    public Pose2d getRobotPose() {
        m_odometry.update(
                Math.toRadians(m_imu.getHeading()),
                m_centralEncoder.getDistance(),
                m_encoderLeft.getDistance(),
                m_encoderRight.getDistance()
        );
        return m_odometry.getPose();

    }

    public void liftToPosition(double position, double power) {
        m_frontLeft.setTargetPosition(position);
        m_frontRight.setTargetPosition(position);
        m_backLeft.setTargetPosition(position);
        m_backRight.setTargetPosition(position);
        m_backLeft.pidWrite(power);
        m_frontRight.pidWrite(power);
        m_frontLeft.pidWrite(power);
        m_backRight.pidWrite(power);
    }

    public void drive(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double allowableRotationError) {
/*
Makes the field into a coordinate system. targetXPosition is the X2 coordinate, and targetYPosition is the Y2 coordinate.
 */
        targetXPosition *= ROBOT_COUNTS_PER_INCH;
        targetYPosition *= ROBOT_COUNTS_PER_INCH;
        double distanceToX = targetXPosition - globalPositionUpdate.returnXCoordinate(); //x distance needed to travel from X1 to X2
        double distanceToY = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
        double robotMoveAngle = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
        double distance = Math.hypot(distanceToX, distanceToY);
        while (distance > allowableDistanceError) {
            //x magnitude vector
            double robotMoveX = calculateX(movementAngle, robotPower);
            //y magnitude vector
            double robotMoveY = calculateY(movementAngle, robotPower);
            if (m_safety == Safety.SWIFT) {
                m_drive.driveFieldCentric(
                        robotMoveX,
                        robotMoveY,
                        0,
                        Math.toRadians(IMUHeading())
                );
            }
        }

        while(robotMoveAngle > allowableRotationError){
            if(m_safety == Safety.SWIFT){
                m_drive.driveRobotCentric(0, 0, movementAngle/160);
            }
        }





    }
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    private double calculateY ( double desiredAngle, double speed){
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public double IMUHeading(){
        return m_imu.getHeading();
    }

    public double returnXPosition(){
        return globalPositionUpdate.returnXCoordinate();
    }
    public double returnYPosition(){
        return globalPositionUpdate.returnYCoordinate();
    }
    public double orientation(){
        return globalPositionUpdate.returnOrientation();
    }
    public boolean alive(){
        return positionThread.isAlive();
    }

    public void reverseEncoders(){
        m_encoderLeft.setInverted(true);
        m_encoderRight.setInverted(true);
        m_centralEncoder.setInverted(true);
    }

    public void start(){
        positionThread.start();
    }
}

