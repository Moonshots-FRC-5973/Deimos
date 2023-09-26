package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive extends Drivetrain {
    //public static final double MOTOR_MAX_POWER = 0.5;
    public static final double BOOST_MULTIPLIER = 3;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;
    private double gyroTarget = 0.0d;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        telemetry.addData("IMU Rotation", "(%.2f, %.2f, %.2f)",
                imu.getXAngle(), imu.getYAngle(), imu.getZAngle());

        isGyroLocked = turn <= Constants.INPUT_THRESHOLD;
        if(isGyroLocked && !isTargetSet) {
            gyroTarget = imu.getZAngle();
            isTargetSet = true;
        } else if(!isGyroLocked) {
            isTargetSet = false;
        }

        double leftFrontBoost = 0;
        double rightFrontBoost = 0;
        double leftBackBoost = 0;
        double rightBackBoost = 0;

        if(isGyroLocked) {
            double gyroDiff = gyroTarget - imu.getZAngle();
            if(gyroDiff <= ANGLE_TOLERANCE) gyroDiff = 0;
            // IF WE ARE ROTATING TO THE RIGHT (gyro difference is more positive)
            // Give the right side more power and drop the left side power

            // IF WE ARE ROTATING TO THE LEFT (gyro difference is more negative)
            // Give the left side more power and drop the right side power

            double turnBoost = BOOST_MULTIPLIER * Math.sin(Math.toRadians(2 * gyroDiff));

            leftFrontBoost -= turnBoost;
            leftBackBoost -= turnBoost;
            rightFrontBoost -= turnBoost;
            rightBackBoost -= turnBoost;
        }

        // Forward / Strafe alignment
        // Uses the accelerometer on the IMU to ensure accurate movement

        // Stop boost if not moving
        if(Math.hypot(forward, strafe) <= Constants.INPUT_THRESHOLD) {
            isGyroLocked = false;
        }

        // I'm tired of figuring out the input problems so the inputs are still in flight stick mode
        // Meaning forward is reversed
        // The boost values should match the turn
        // Since the drive is a diamond wheel pattern instead of an X, it reverses the strafe.
        drive(
                -forward + strafe - (turn + leftFrontBoost),
                forward + strafe - (turn + rightFrontBoost),
                forward + strafe - (turn + leftBackBoost),
                forward - strafe - (turn + rightBackBoost)
        );
    }

    @Override
    protected void drive(double m1, double m2, double m3, double m4) {
        telemetry.addData("Motors", "(%.2f, %.2f, %.2f, %.2f)", m1, m2, m3, m4);
        leftFront.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightFront.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        leftBack.setPower(Range.clip(m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightBack.setPower(Range.clip(m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    @Override
    public void turnRobotToAngle(double target) {

    }
}