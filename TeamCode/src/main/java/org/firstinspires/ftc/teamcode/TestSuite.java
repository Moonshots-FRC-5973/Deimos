package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.systems.CascadeArm;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.wrappers.IMU;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Test Suite")
public class TestSuite extends LinearOpMode {

    private Camera camera;
    private long time;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean gamepad1APressed = false;
    private boolean gamepad1BPressed = false;

    private enum DisplayMode {
        IMU_DISPLAY,
        SENSOR_DISPLAY,
        CV_DISPLAY,
        HARDWARE_DISPLAY,
    }

    private DisplayMode mode = DisplayMode.IMU_DISPLAY;
    private double position = Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        /*
        DcMotor motor = hardwareMap.get(DcMotor.class, "rlMotor");
        DcMotor hold = hardwareMap.get(DcMotor.class, "rrMotor");
        hold.setTargetPosition(0);
        hold.setPower(0.5d);
        hold.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(!isStopRequested()) {
            telemetry.addData("Encoder", motor.getCurrentPosition());
            motor.setTargetPosition((int)(position));
            motor.setPower(0.5d);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.a && !gamepad1APressed) {
                position += Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV;
            }
            if(gamepad1.b && !gamepad1BPressed) {
                position -= Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV;
            }

            gamepad1APressed = gamepad1.a;
            gamepad1BPressed = gamepad1.b;

            telemetry.update();
        }



        CascadeArm arm = new CascadeArm(hardwareMap, telemetry);

        waitForStart();

        while(!isStopRequested()) {
            if (gamepad1.a) {
                arm.raiseArm();
            }
            if (gamepad1.b) {
                arm.lowerArm();
            }
        }

         */

        Camera camera = new Camera(hardwareMap, telemetry);

        while (opModeInInit()) {
            telemetry.addData("camerafps", camera.getFps());
        }
    }
}
