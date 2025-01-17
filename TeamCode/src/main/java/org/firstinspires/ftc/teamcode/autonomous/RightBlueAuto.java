package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.systems.Shoulder;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Right Blue Autonomous")
public class RightBlueAuto extends LinearOpMode {
    // SUBSYSTEMS
    private MecanumDrive drive;
    private Shoulder shoulder;

    // SENSORS
    private DistanceSensor rearDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;

    // SUBSYSTEMS THAT ARE JUST SENSORS
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
        shoulder = new Shoulder(hardwareMap, null);

        rearDistance = new DistanceSensor(hardwareMap, "rear");
        leftDistance = new DistanceSensor(hardwareMap, "left");
        rightDistance = new DistanceSensor(hardwareMap, "right");

        camera = new Camera(hardwareMap, telemetry);

        //makes arm not slam on the ground on start of autonomous
        ElapsedTime rt = new ElapsedTime();
        shoulder.move(0.1);

        while(opModeInInit() && rt.seconds() <= 3) {
            wait();
        }

        shoulder.move(0);

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

        shoulder.open();
        shoulder.wristTo(0.5);

        sleep(3000);

        shoulder.close();
        shoulder.wristTo(1.0);

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
                drive.drive( -activeDetection.ftcPose.x / 5, 0, -Math.toRadians(activeDetection.ftcPose.yaw));
                if(Math.abs(activeDetection.ftcPose.x) <= 0.1 && Math.abs(Math.toRadians(activeDetection.ftcPose.yaw)) <= 0.1) {
                    break;
                }
            }
            // PX, PY, AZ
        }

        while(opModeIsActive() && rearDistance.getDistance() >= 8) {
            drive.drive(0.0, -0.15, 0.0);
        }

        while(opModeIsActive() && rightDistance.getDistance() <= 42) {
            drive.drive(-1.0, 0.0, 0.0);
        }

        while(opModeIsActive() && rearDistance.getDistance() >= 1) {
            drive.drive(0.0, -0.2, 0.0);
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
