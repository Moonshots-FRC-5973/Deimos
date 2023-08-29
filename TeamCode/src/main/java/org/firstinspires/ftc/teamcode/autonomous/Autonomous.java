package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.SwerveDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap, null);
        waitForStart();
        drive.setDistanceToTravel(12);
        drive.turnRobotByDegree(90);
    }
}
