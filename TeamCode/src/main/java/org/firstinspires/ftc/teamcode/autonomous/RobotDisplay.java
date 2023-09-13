package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;

@Autonomous(name = "Robot Display Autonomous")
public class RobotDisplay extends LinearOpMode {
    public static final double TIME_FOR_CYCLE = 5 / Math.PI;
    public static final double MOTOR_POWER = 1d;

    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, null);
        runtime.reset();
        while(opModeInInit()) {
            double forward, strafe;

            while(runtime.seconds() / TIME_FOR_CYCLE <= 2* Math.PI && !isStopRequested()) {
                forward = MOTOR_POWER * Math.sin(runtime.seconds() / TIME_FOR_CYCLE);
                strafe = MOTOR_POWER * Math.sin(runtime.seconds() / TIME_FOR_CYCLE);

                drive.drive(forward,
                        strafe,
                        0.0d
                );
                telemetry.addData("Power","(%.2f, %.2f)", forward, strafe);
                telemetry.update();
            }

            while(runtime.seconds() / TIME_FOR_CYCLE <= 4 * Math.PI && !isStopRequested()) {
                forward = MOTOR_POWER * Math.sin(runtime.seconds() / TIME_FOR_CYCLE);
                strafe = MOTOR_POWER * Math.cos(runtime.seconds() / TIME_FOR_CYCLE);

                drive.drive(forward,
                        strafe,
                        0.0d
                );
                telemetry.addData("Power","(%.2f, %.2f)", forward, strafe);
                telemetry.update();
            }

            while(runtime.seconds() / TIME_FOR_CYCLE >= 6 * Math.PI && !isStopRequested()) {
                forward = MOTOR_POWER * Math.cos(runtime.seconds() / TIME_FOR_CYCLE);
                strafe = MOTOR_POWER * Math.sin(runtime.seconds() / TIME_FOR_CYCLE);

                drive.drive(forward,
                        strafe,
                        0.0d
                );
                telemetry.addData("Power","(%.2f, %.2f)", forward, strafe);
                telemetry.update();
            }

            while(runtime.seconds() / TIME_FOR_CYCLE >= 8 * Math.PI && !isStopRequested()) {
                forward = MOTOR_POWER * Math.cos(runtime.seconds() / TIME_FOR_CYCLE);
                strafe = MOTOR_POWER * Math.cos(runtime.seconds() / TIME_FOR_CYCLE);

                drive.drive(forward,
                        strafe,
                        0.0d
                );
                telemetry.addData("Power","(%.2f, %.2f)", forward, strafe);
                telemetry.update();
            }

            runtime.reset();
        }
    }

}
