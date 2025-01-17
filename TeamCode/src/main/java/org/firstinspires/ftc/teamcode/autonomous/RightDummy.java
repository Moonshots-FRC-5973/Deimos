package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Shoulder;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Right Dummy")
public class RightDummy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shoulder shoulder = new Shoulder(hardwareMap, null);

        MecanumDrive drive = new MecanumDrive(hardwareMap, null);

        waitForStart();
        if(opModeIsActive())
            drive.drive(0.0, 0.3, 0.0);
        while(opModeIsActive())
            wait();

        drive.stop();
    }
}
