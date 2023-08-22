package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.SwerveDrive;
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
/*
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new Camera( hardwareMap, telemetry);

        while(opModeInInit()) {
            telemetry.addData("UPS", 1 / runtime.seconds());
            runtime.reset();
            telemetry.addData("CV Frame Count", camera.getFrameCount());
            telemetry.addData("CV FPS", camera.getFps());
            telemetry.addData("CV Frame Time", String.format("%.2f ms", camera.getTotalFrameTime()));
            telemetry.addData("CV Pipeline Time", String.format("%.2f ms", camera.getPipelineTime()));
            telemetry.addData("CV Overhead Time", String.format("%.2f ms", camera.getOverheadTime()));
            telemetry.addData("CV Theoretical Max FPS", camera.getCurrentPipelineMaxFps());
            telemetry.addData("CV Viewport Enabled", camera.getPipeline().isViewportPaused());
            telemetry.update();
        }
    }

 */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SwerveDrive drive = new SwerveDrive(hardwareMap, telemetry);

        while(opModeInInit()) {
            telemetry.addData("UPS", 1 / runtime.seconds());
            runtime.reset();
            IMU imu = drive.getIMU();
            telemetry.addData("IMU Angle", imu.getAngle().toString());
            //telemetry.addData("IMU Acceleration", imu.getAcceleration());
            telemetry.addData("IMU Velocity", imu.getVelocity().toString());
            telemetry.addData("IMU Position", imu.getPosition().toString());
            telemetry.update();
        }
    }
}
