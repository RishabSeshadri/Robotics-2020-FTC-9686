package org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.opmodes;
import org.incoherentdemonicscreeching.firstinspires.ftc.teamcode.hardware.skystoneRobocop;

import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "AutoCop 2: The Electric Boogaloo")
public class autocop extends LinearOpMode {

    private skystoneRobocop robocop = new skystoneRobocop(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robocop.setSafetyMode(Safety.SWIFT);

        telemetry.addData("Robot Position", robocop.getRobotPose());

        waitForStart();

        //23.625 inches per foam square

        //drive and intake skystone
        robocop.drive(60 / (4 * Math.PI) * 383.6, 0, 0);
        robocop.intake();
        robocop.drive( -(60 / (4 * Math.PI) * 383.6), 0, 0);

        //turn to move under bridge
        robocop.drive(0, 0, 90);

        //move under bridge
        robocop.drive(0,70 / (4 * Math.PI) * 383.6, 0);

        //turn to face foundation
        robocop.drive(0, 0, -90);

        //drive and deposit skystone
        robocop.drive(60 / (4 * Math.PI) * 383.6, 0, 0);
        robocop.outtake();
        robocop.drive( -(60 / (4 * Math.PI) * 383.6), 0, 0);
    }
}
