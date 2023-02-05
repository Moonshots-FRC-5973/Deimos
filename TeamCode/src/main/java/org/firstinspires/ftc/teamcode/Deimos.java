package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@TeleOp(name="Deimos")
public class Deimos extends LinearOpMode {
    private MecanumDrive drive;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private boolean gp1aPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init (runs once)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, telemetry);

        // Init Loop (runs until stop button or start button is pressed)
        while(!opModeIsActive()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("G2LS", "(%f, %f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("G2RS", "(%f, %f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
        }
        // Start (runs once)

        // Main (runs until stop is pressed)
        while(opModeIsActive()) {
            telemetry.addData("UPS", 1000 / elapsedTime.milliseconds());
            // Driver 1: Responsible for drivetrain and movement
            driver1Inputs();
            // Driver 2: Responsible for the subsystem attachment
            driver2Inputs();
            telemetry.update();
        }
        // Stop (runs once)
        drive.stop();
    }

    private void driver1Inputs() {
        // DPad inputs, checking for overload; control for the drivetrain to rotate the robot
        boolean turnUp = (gamepad1.dpad_up && !gamepad1.dpad_down);
        boolean turnDown = (gamepad1.dpad_down && !gamepad1.dpad_up);
        boolean turnLeft = (gamepad1.dpad_left && !gamepad1.dpad_right);
        boolean turnRight = (gamepad1.dpad_right && !gamepad1.dpad_left);

        if(turnUp) {
            if(turnRight) {
                telemetry.addData("Drive", "Turning to -45");
                drive.turnRobotToAngle(-45);
            } else if(turnLeft) {
                telemetry.addData("Drive", "Turning to 45");
                drive.turnRobotToAngle(45);
            } else {
                telemetry.addData("Drive", "Turning to 0");
                drive.turnRobotToAngle(0);
            }
        } else if(turnDown) {
            if (turnRight) {
                telemetry.addData("Drive", "Turning to -135");
                drive.turnRobotToAngle(-135);
            } else if (turnLeft) {
                telemetry.addData("Drive", "Turning to 135");
                drive.turnRobotToAngle(135);
            } else {
                telemetry.addData("Drive", "Turning to 180");
                drive.turnRobotToAngle(180);
            }
        } else if(turnRight) {
            telemetry.addData("Drive", "Turning to -90");
            drive.turnRobotToAngle(-90);
        } else if(turnLeft) {
            telemetry.addData("Drive", "Turning to 90");
            drive.turnRobotToAngle(90);
        }
        else {
            telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
            double forward = -gamepad1.left_stick_y; // flight stick inversion
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // DEADZONES
            if (Math.abs(forward) <= Constants.INPUT_THRESHOLD) forward = 0.0d;
            if (Math.abs(strafe) <= Constants.INPUT_THRESHOLD)  strafe = 0.0d;
            if (Math.abs(rotate) <= Constants.INPUT_THRESHOLD) rotate = 0.0d;

            drive.drive(forward, strafe, rotate);
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

    private void driver2Inputs() {

    }
}
