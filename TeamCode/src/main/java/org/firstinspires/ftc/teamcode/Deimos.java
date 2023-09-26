package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Deimos")
public class Deimos extends LinearOpMode {
    public static final double APRIL_TAG_DISTANCE_TARGET = 20;
    public static final double APRIL_TAG_PRECISION = 5;
    public static final double APRIL_TAG_MAX_SPEED = 0.4;

    private enum AprilTagToAlign {
        LEFT,
        CENTER,
        RIGHT
    }

    private MecanumDrive drive;
    private Camera camera;
    private double lastTime = 0.0d;
    private final ElapsedTime elapsedTime = new ElapsedTime();

    private boolean gp1aPressed = false;
    
    @Override
    public void runOpMode() {
        // Init (runs once)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, telemetry);

        camera = new Camera(hardwareMap, telemetry);

        // Init Loop (runs until stop button or start button is pressed)
        while(opModeInInit()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("G2LS", "(%f, %f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("G2RS", "(%f, %f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
        }
        // Start (runs once)
        telemetry.addData("Status", "Started");
        telemetry.update();
        elapsedTime.reset();

        // Main (runs until stop is pressed)
        while(opModeIsActive()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("UPS", 1 / (elapsedTime.seconds() - lastTime));
            lastTime = elapsedTime.seconds();
            //
            // Driver 1: Responsible for drivetrain and movement
            driver1Inputs();
            // Driver 2: Responsible for the subsystem attachment
            driver2Inputs();
            //



            telemetry.update();
        }
        // Stop (runs once)
        drive.stop();
    }

    /**
     * Driver 1: Solely responsible for the control of the drivetrain;
     * this function never changes and should not be changed unless addition of a new
     * feature is required.
     */
    private void driver1Inputs() {
        // DPad inputs, checking for overload; control for the drivetrain to rotate the robot
        boolean dpadUp = (gamepad1.dpad_up && !gamepad1.dpad_down);
        boolean dpadDown = (gamepad1.dpad_down && !gamepad1.dpad_up);
        boolean dpadLeft = (gamepad1.dpad_left && !gamepad1.dpad_right);
        boolean dpadRight = (gamepad1.dpad_right && !gamepad1.dpad_left);

        if(dpadLeft && !(dpadUp || dpadRight)) {
            alignToAprilTag(AprilTagToAlign.LEFT);
        } else if(dpadUp && !(dpadLeft || dpadRight)) {
            alignToAprilTag(AprilTagToAlign.CENTER);
        } else if(dpadRight && !(dpadUp || dpadLeft)) {
            alignToAprilTag(AprilTagToAlign.RIGHT);
        } else {
            telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // DEADZONES
            if (Math.abs(forward) <= Constants.INPUT_THRESHOLD) forward = 0.0d;
            if (Math.abs(strafe) <= Constants.INPUT_THRESHOLD)  strafe = 0.0d;
            if (Math.abs(turn) <= Constants.INPUT_THRESHOLD) turn = 0.0d;

            drive.drive(forward, strafe, turn);
        }

        if(gamepad1.a && !gp1aPressed && !gamepad1.start) {
            drive.toggleFieldCentric();
        }

        if(drive.getFieldCentric())
            telemetry.addData("Mode", "Field Centric");
        else
            telemetry.addData("Mode", "Robot Centric");

        gp1aPressed = gamepad1.a;
    }

    /**
     * Driver 2: responsible for any subsystem attachments we may have.
     * This function's implementation changes quickly and rapidly every year.
     */
    private void driver2Inputs() {
    }

    private void alignToAprilTag(AprilTagToAlign alignment) {
        switch (alignment) {
            case LEFT: telemetry.addData("Aligning To", "Left"); break;
            case CENTER: telemetry.addData("Aligning To", "Center"); break;
            case RIGHT: telemetry.addData("Aligning To", "Right"); break;
        }
        // Get AprilTags
        AprilTagDetection correctTag;
        List<AprilTagDetection> detections = camera.getDetections();
        if(detections.size() == 0) {
            drive.stop();
            telemetry.addData("Detections", "No AprilTags found");
            return;
        }

        telemetry.addData("Detections", detections.size());

        do {
            AprilTagDetection detection = detections.get(0);
            telemetry.addData("Tracking XYZ", "(%.2f, %.2f, %.2f)",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
            telemetry.addData("Tracking YPR", "(%.2f, %.2f, %.2f)",
                    detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);

            // According to the alignment, we need to find if one of the tags is the correct one,
            // or if we have to adjust to find it.
            if(alignment == AprilTagToAlign.LEFT) {
                double turn = Math.toRadians(-detection.ftcPose.yaw / (3 * APRIL_TAG_PRECISION));
                if(detection.metadata.name.toLowerCase().contains("left")) {
                    telemetry.addData("Position", "Finalizing");
                    // Treat the flight stick inversion
                    double forward = Range.clip((detection.ftcPose.y - APRIL_TAG_DISTANCE_TARGET) / APRIL_TAG_PRECISION, -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);
                    // Reversed since the camera is on the back of the robot
                    double strafe = Range.clip((-detection.ftcPose.x) / APRIL_TAG_PRECISION, -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);

                    if (
                            Math.abs(forward) <= Constants.INPUT_THRESHOLD &&
                                    Math.abs(strafe) <= Constants.INPUT_THRESHOLD &&
                                    Math.abs(turn) <= Constants.INPUT_THRESHOLD
                    ) {
                        drive.stop();
                        telemetry.addData("Movement", "Done");
                        return;
                    }

                    telemetry.addData("Movement", "(%.2f, %.2f, %.2f)",
                            forward, strafe, turn);

                    drive.drive(forward, strafe, turn);
                    break;
                } else {
                    telemetry.addData("Position", "Wrong Tag");
                    // Treat the flight stick, even if it's the wrong tag, it'll still be correct to adjust to
                    double forward = Range.clip((detection.ftcPose.y - APRIL_TAG_DISTANCE_TARGET) / APRIL_TAG_PRECISION, -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);
                    // Reversed since the camera is on the back of the robot
                    double strafe = APRIL_TAG_MAX_SPEED;

                    telemetry.addData("Movement", "(%.2f, %.2f, %.2f)",
                            forward, strafe, turn);

                    drive.drive(forward, strafe, turn);
                    break;
                }
            }

            telemetry.update();
            detections = camera.getDetections();
        } while(detections.size() != 0 && !gamepad1.dpad_down && !isStopRequested());
    }
}
