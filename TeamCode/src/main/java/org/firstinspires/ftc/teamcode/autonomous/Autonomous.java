package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.SwerveDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private SwerveDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SwerveDrive(hardwareMap, null);
        waitForStart();
        drive.setDistanceToTravel(12);
        sleep(2000);
        drive.setWheelAngle(90);
        runtime.reset();
        while(runtime.seconds() <= 2) {
            drive.drive(0.0d, 0.0d, 0.0d);
        }
        drive.setDistanceToTravel(12);
        drive.setWheelAngle(180);
        runtime.reset();
        while(runtime.seconds() <= 2) {
            drive.drive(0.0d, 0.0d, 0.0d);
        }
    }
}
