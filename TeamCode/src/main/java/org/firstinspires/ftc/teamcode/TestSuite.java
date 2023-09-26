package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;

import org.firstinspires.ftc.teamcode.systems.CascadeArm;
import org.firstinspires.ftc.teamcode.systems.DroneLauncher;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.wrappers.IMU;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Test Suite")
public class TestSuite extends LinearOpMode {

    //private Camera camera;
    private long time;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean gamepad1APressed = false;

    private enum DisplayMode {
        IMU_DISPLAY,
        SENSOR_DISPLAY,
        CV_DISPLAY,
        HARDWARE_DISPLAY,
    }

    private DisplayMode mode = DisplayMode.IMU_DISPLAY;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while(opModeInInit()) {
            telemetry.addData("UPS", 1 / runtime.seconds());
            runtime.reset();
            telemetry.update();
            gamepad1APressed = gamepad1.a;
        }

        waitForStart();

/*
        Camera camera = new Camera(hardwareMap, telemetry);

        while (opModeInInit()) {
            telemetry.addData("camerafps", camera.getFps());
        }
        */

        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap, telemetry);
        while(! isStopRequested()){
            if(gamepad1.a){
                droneLauncher.lauchDrone();

            }
            else droneLauncher.stopDrone();

        }


    }
}
