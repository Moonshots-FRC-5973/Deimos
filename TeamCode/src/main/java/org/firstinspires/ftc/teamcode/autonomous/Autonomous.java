package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private MecanumDrive drive;

    private DistanceSensor rearDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, telemetry);

        rearDistance = hardwareMap.get(DistanceSensor.class, "rear_distance");

        waitForStart();

        while(rearDistance.getDistance(DistanceUnit.INCH) <= 30 && opModeIsActive()) {
            drive.drive(1.0, 0.0, 0.0);
        }

        while(drive.getIMU().getZAngle() <= 90) {
            drive.drive(0.0, 0.0, 1.0);
        }

    }
}
