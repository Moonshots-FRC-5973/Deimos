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
import org.firstinspires.ftc.teamcode.wrappers.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.IOException;

@TeleOp(name = "Test Suite")
public class TestSuite extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        PIDController controller = new PIDController(telemetry, "theta");
        telemetry.addData("IMU Rotation", "(%.2f, %.2f, %.2f)",
                drive.getIMU().getXAngle(), drive.getIMU().getYAngle(), drive.getIMU().getZAngle());
        double turnStrength = controller.getPIDControlledValue(Math.toRadians(drive.getIMU().getZAngle()), Math.PI / 2);
        telemetry.update();
        waitForStart();

        try {
            controller.resetPID();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        do {
            telemetry.addData("turnStrength", turnStrength);
            telemetry.update();
            drive.drive(0.0d, 0.0d, turnStrength);
            turnStrength = controller.getPIDControlledValue(Math.toRadians(drive.getIMU().getZAngle()), Math.PI / 2);
        } while(turnStrength >= 0.05);
        try {
            controller.resetPID();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        drive.stop();
    }
}
