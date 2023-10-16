package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.teamcode.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

@TeleOp(name = "Capability Viewer")
public class CapsViewer extends LinearOpMode {
    private Drivetrain drive;
    private DistanceSensor rearDistance;
    private DistanceSensor leftDistance;

    private boolean next = false;
    private boolean prev = false;
    private boolean runDrive = false;
    private boolean lastUpDPad = false;

    public enum DriveType {
        SWERVE,
        MECANUM,
        NONE
    }

    public enum Page {
        IMU_VIEW,
        SENSOR_VIEW,
        CV_VIEW
    }

    private DriveType driveType = DriveType.NONE;

    private Page currentPage = Page.IMU_VIEW;
    private Page nextPage = Page.SENSOR_VIEW;
    private Page prevPage = Page.CV_VIEW;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IMU imu;
        try {
            try {
                drive = new SwerveDrive(hardwareMap, telemetry);
                driveType = DriveType.SWERVE;

            } catch (Exception e) {
                drive = new MecanumDrive(hardwareMap, telemetry);
                driveType = DriveType.MECANUM;
            }
            imu = drive.getIMU();
        } catch (Exception e) {
            imu = new IMU(hardwareMap, new com.qualcomm.robotcore.hardware.IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            ));
            driveType = DriveType.NONE;
        }

        rearDistance = new DistanceSensor(hardwareMap, "rear");
        leftDistance = new DistanceSensor(hardwareMap, "left");

        waitForStart();

        while(opModeIsActive()) {
            switch(driveType) {
                case SWERVE:
                    telemetry.addData("Drive Type", "Swerve");
                    break;
                case MECANUM:
                    telemetry.addData("Drive Type", "Mecanum");
                    break;
                case NONE:
                    telemetry.addData("Drive Type", "Not Found");
                    break;
            }

            switch(currentPage) {
                case IMU_VIEW:
                    telemetry.addData("Page", "IMU");
                    telemetry.addData("Angle", "(%.2f, %.2f, %.2f)", imu.getXAngle(), imu.getYAngle(), imu.getZAngle());
                    telemetry.addData("Position", "(%.2f, %.2f, %.2f)", imu.getXPosition(), imu.getYPosition(), imu.getZPosition());
                    telemetry.addData("Velocity", "(%.2f, %.2f, %.2f)", imu.getXVelocity(), imu.getYVelocity(), imu.getZVelocity());
                    prevPage = Page.CV_VIEW;
                    nextPage = Page.SENSOR_VIEW;
                    break;
                case SENSOR_VIEW:
                    telemetry.addData("Page", "Sensors");
                    telemetry.addData("Rear Distance Sensor", rearDistance.getDistance());
                    telemetry.addData("Left Distance Sensor", leftDistance.getDistance());
                    prevPage = Page.IMU_VIEW;
                    nextPage = Page.CV_VIEW;
                    break;
                case CV_VIEW:
                    telemetry.addData("Page", "CV");
                    prevPage = Page.SENSOR_VIEW;
                    nextPage = Page.IMU_VIEW;
                    break;
            }

            if(drive != null && runDrive) {
                telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
                double forward = gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // DEADZONES
                if (Math.abs(forward) <= Constants.INPUT_THRESHOLD) forward = 0.0d;
                if (Math.abs(strafe) <= Constants.INPUT_THRESHOLD)  strafe = 0.0d;
                if (Math.abs(turn) <= Constants.INPUT_THRESHOLD) turn = 0.0d;

                drive.drive(forward, strafe, turn);
            } else if(drive != null) {
                drive.stop();
            }

            telemetry.update();

            if(gamepad1.right_bumper && ! next) {
                currentPage = nextPage;
            } else if(gamepad1.left_bumper && ! prev) {
                currentPage = prevPage;
            }

            next = gamepad1.right_bumper;
            prev = gamepad1.left_bumper;

            if(gamepad1.dpad_up && !lastUpDPad) {
                runDrive = !runDrive;
            }
            lastUpDPad = gamepad1.dpad_up;
        }
    }




}
