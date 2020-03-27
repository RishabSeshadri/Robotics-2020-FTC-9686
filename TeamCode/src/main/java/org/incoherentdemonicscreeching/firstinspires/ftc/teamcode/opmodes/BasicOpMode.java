package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.motors.*;

// my first op mode! (not really lol)

/**
 * Representation of a mecanum drive with Yellow Jackets at a 13.7:1 gearbox ratio
 * and 1:1 belt
 */
@TeleOp(name="Simple Mecanum Bot")
public class BasicOpMode extends OpMode {

    private YellowJacket435 m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDrive m_drive;

    private RevIMU m_imu;

    @Override
    public void init() {
        m_frontLeft = new YellowJacket435(hardwareMap, "fl");
        m_frontRight = new YellowJacket435(hardwareMap, "fr");
        m_backLeft = new YellowJacket435(hardwareMap, "bl");
        m_backRight = new YellowJacket435(hardwareMap, "br");

        m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

        m_imu = new RevIMU(hardwareMap);
    }

    @Override
    public void loop() {
        m_drive.driveFieldCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                Math.toRadians(m_imu.getHeading())
        );
    }

}
