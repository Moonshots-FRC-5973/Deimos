package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.Camera;

import java.util.List;

@TeleOp(name = "Test Suite")
public class TestSuite extends LinearOpMode {

    private Camera camera;
    private long time;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new Camera(hardwareMap, telemetry);


        while(opModeInInit()) {
            telemetry.addData("UPS", 1 / runtime.seconds());
            runtime.reset();
            telemetry.addData("CV Frame Count", camera.getFrameCount());
            telemetry.addData("CV FPS", camera.getFps());
            telemetry.addData("CV Frame Time", String.format("%.2f ms", camera.getTotalFrameTime()));
            telemetry.addData("CV Pipeline Time", String.format("%.2f ms", camera.getPipelineTime()));
            telemetry.addData("CV Overhead Time", String.format("%.2f ms", camera.getOverheadTime()));
            telemetry.addData("CV Theoretical Max FPS", camera.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}
