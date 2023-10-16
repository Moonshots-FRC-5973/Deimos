package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.wrappers.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private MecanumDrive drive;

    private DistanceSensor rearDistance;
    private DistanceSensor leftDistance;

    private Camera camera;

    private enum TargetPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, null);

        rearDistance = new DistanceSensor(hardwareMap, "rear");
        leftDistance = new DistanceSensor(hardwareMap, "left");

        camera = new Camera(hardwareMap, telemetry);

        waitForStart();

        while(rearDistance.getDistance(DistanceUnit.INCH) <= 18 && opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", drive.getIMU().getZAngle());
            telemetry.update();
            drive.drive(-0.3, 0.0, 0.0);
        }
        drive.stop();

        while(rearDistance.getDistance(DistanceUnit.INCH) >= 8 && opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", drive.getIMU().getZAngle());
            telemetry.update();
            drive.drive(0.0, 0.0, 0.3);
        }
        drive.stop();

        double targetAngle = drive.getIMU().getZAngle() + 180;
        if(targetAngle > 180) {
            targetAngle -= 360;
        }

        while(Math.abs(targetAngle - drive.getIMU().getZAngle()) >= 1 && opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", drive.getIMU().getZAngle());
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            drive.drive(0.0, 0.0, -0.3);
        }
        drive.stop();

        sleep(3000);

        while(Math.abs(drive.getIMU().getZAngle()) >= 1 && opModeIsActive()) {
            drive.drive(0.0, 0.0, Math.toRadians(drive.getIMU().getZAngle()));
        }
        drive.stop();

        while((rearDistance.getDistance() >= 6 || leftDistance.getDistance() >= 24) && opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance());
            telemetry.addData("Rear Distance", leftDistance.getDistance());
            telemetry.addData("IMU Angle", drive.getIMU().getZAngle());
            telemetry.addData("Inputs", "(%.2f, %.2f, %.2f)", (rearDistance.getDistance() - 6),
                    -(leftDistance.getDistance() - 24),
                    Math.toRadians(drive.getIMU().getZAngle()));
            telemetry.update();


            drive.drive(
                    Range.clip((rearDistance.getDistance() - 6), -1, 1) / 4,
                    Range.clip((-(leftDistance.getDistance() - 24)), -1, 1) / 4,
                    Math.toRadians(drive.getIMU().getZAngle()));
        }
        drive.stop();
        sleep(100);
        drive.toggleFieldCentric();
        while(Math.abs(drive.getIMU().getZAngle() + 90) >= 1 && opModeIsActive()) {
            drive.drive(-0.1, 0.0, Math.toRadians(drive.getIMU().getZAngle() + 90));
        }
        drive.toggleFieldCentric();
        drive.stop();

        while(camera.getDetections().size() == 0 && opModeIsActive()) {
            drive.drive(-0.2, 0.0, 0.0);
        }
        drive.stop();

        TargetPosition targetPosition;

        if(targetAngle <= -22.5) {
            targetPosition = TargetPosition.RIGHT;
        } else if(targetAngle >= 22.5) {
            targetPosition = TargetPosition.LEFT;
        } else {
            targetPosition = TargetPosition.CENTER;
        }

        while(opModeIsActive()) {
            List<AprilTagDetection> detections = camera.getDetections();
            AprilTagDetection activeDetection = null;
            for(AprilTagDetection detection : detections) {
                switch(targetPosition) {
                    case LEFT:
                        if(detection.metadata.name.toLowerCase().contains("left")) {
                            activeDetection = detection;
                        }
                        break;
                    case CENTER:
                        if(detection.metadata.name.toLowerCase().contains("center")) {
                            activeDetection = detection;
                        }
                        break;
                    case RIGHT:
                        if(detection.metadata.name.toLowerCase().contains("right")) {
                            activeDetection = detection;
                        }
                        break;
                }
            }
            if(activeDetection == null) {
                drive.drive(-0.15, 0.0, 0.0);
            } else {
                telemetry.addData("Detection (X, Y, Theta", "(%.2f, %.2f, %.2f", activeDetection.ftcPose.x, activeDetection.ftcPose.y, activeDetection.ftcPose.yaw);
                telemetry.update();
                double strafe = 4 - activeDetection.ftcPose.y;
                if(rearDistance.getDistance() <= 4) strafe = 0;
                drive.drive( -activeDetection.ftcPose.x / 5, strafe, Math.toRadians(activeDetection.ftcPose.yaw));
            }

            // PX, PY, AZ
        }
        /*
        PIDController thetaController = new PIDController(telemetry, "theta");
        double turnStrength = thetaController.getPIDControlledValue(Math.toRadians(drive.getIMU().getZAngle()), targetAngle);

        while(turnStrength >= Constants.INPUT_THRESHOLD && opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", drive.getIMU().getZAngle());
            telemetry.update();
            drive.drive(0.0d, 0.0d, turnStrength);
            turnStrength = thetaController.getPIDControlledValue(Math.toRadians(drive.getIMU().getZAngle()), targetAngle);
        }

         */
    }
}
