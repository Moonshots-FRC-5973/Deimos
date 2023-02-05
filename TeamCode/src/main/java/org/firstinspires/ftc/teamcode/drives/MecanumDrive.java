package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive extends Drivetrain {

    // HARDWARE
    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    // GYRO TRACKERS
    private double gyroTarget;

    // BOOLEAN TOGGLES
    private boolean gyroLocked = false;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));

        // CONFIGURE HARDWARE
        leftFrontDrive = hardwareMap.get(DcMotor.class, "flMotor"); // Florida Motor
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBackDrive = hardwareMap.get(DcMotor.class, "blMotor"); // Saint Barthélemy Motor
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFrontDrive = hardwareMap.get(DcMotor.class, "frMotor"); // French Motor
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rightBackDrive = hardwareMap.get(DcMotor.class,"brMotor"); // Brazilian Motor
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Consider conditions and calculate the motion for the drivetrain
     * @param forward a double to move in the forward direction
     * @param strafe a double to move in the horizontal direction
     * @param turn a double representing the speed to change the heading
     */
    public void drive(double forward, double strafe, double turn) {
        if(telemetry != null) {
            telemetry.addData("IMU", "Accel(%.3f, %.3f, %.3f)", imu.getXVelocity(), imu.getYVelocity(), imu.getZVelocity());
        }

        if (isFieldCentric) {
            // Learn more: https://www.geogebra.org/m/fmegkksm
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(imu.getZAngle())) - strafe * Math.sin(Math.toRadians(imu.getZAngle()));
            strafe = temp * Math.sin(Math.toRadians(imu.getZAngle())) + strafe * Math.cos(Math.toRadians(imu.getZAngle()));
        }

        // ROTATE
        // RIGHT STICK DISABLES FORWARD/STRAFE
        if (Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            drive(turn, -turn, -turn, turn);
            gyroLocked = false;
            return;
        }

        // -------------------
        // GYRO LOCK!
        // If we're not turning, lock our gyro to track our intended heading
        if (!gyroLocked &&
                (Math.abs(forward) >= Constants.INPUT_THRESHOLD || Math.abs(strafe) >= Constants.INPUT_THRESHOLD)) {
            gyroLocked = true;
            gyroTarget = imu.getZAngle();
        }
        // avoid freak-out: kill gyroLock if movement is over
        else if (gyroLocked &&
                ((Math.abs(forward) < Constants.INPUT_THRESHOLD || Math.abs(strafe) < Constants.INPUT_THRESHOLD))){
            gyroLocked = false;
        }
        double gyroError = gyroTarget - imu.getZAngle();

        double frontLeftBoost = 0.0d;
        double frontRightBoost = 0.0d;
        double backLeftBoost = 0.0d;
        double backRightBoost = 0.0d;

        if(gyroLocked) {
            // if gyroError is pos, we're rotating to the left, so left side should get more power
            if(gyroError > ANGLE_TOLERANCE) {
                frontLeftBoost = ((gyroError % 180) / 180);
                backLeftBoost = ((gyroError % 180) / 180);
                frontRightBoost = -((gyroError % 180) / 180);
                backRightBoost = -((gyroError % 180) / 180);
            }
            // if gyroError is neg, we're rotating right, so right side should get more power
            else if (gyroError < ANGLE_TOLERANCE){
                frontLeftBoost = -((gyroError % 180) / 180);
                backLeftBoost = -((gyroError % 180) / 180);
                frontRightBoost = ((gyroError % 180) / 180);
                backRightBoost = ((gyroError % 180) / 180);
            }
        }
        // END GYRO-LOCK
        // --------------------

        if(telemetry != null) {
            telemetry.addData("Gyro Locked", gyroLocked);
            telemetry.addData("Gyro Target", gyroTarget);
            telemetry.addData("Error", gyroError);
        }

        drive(
                -forward - strafe ,
                forward - strafe ,
                -forward - strafe ,
                -forward + strafe
        );

    }

    /**
     * Execute motion on the drivetrain
     * @param leftFront The power to give the left front motor
     * @param leftRear The power to give the left rear motor
     * @param rightFront The power to give the right front motor
     * @param rightRear The power to give the right rear motor
     */
    public void drive(double leftFront, double leftRear, double rightFront, double rightRear) {
        if(telemetry != null) {
            telemetry.addData("leftFront", leftFront);
            telemetry.addData("leftRear", leftRear);
            telemetry.addData("rightFront", rightFront);
            telemetry.addData("rightRear", rightRear);
        }

        leftFrontDrive.setPower(Range.clip(leftFront, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(leftRear, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightFrontDrive.setPower(Range.clip(rightFront, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightBackDrive.setPower(Range.clip(rightRear, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    @Override
    public void resetWheels() {
        drive(
                -leftFrontDrive.getCurrentPosition() / ENCODER_COUNTS_PER_REV,
                -leftBackDrive.getCurrentPosition() / ENCODER_COUNTS_PER_REV,
                -rightFrontDrive.getCurrentPosition() / ENCODER_COUNTS_PER_REV,
                -rightBackDrive.getCurrentPosition() / ENCODER_COUNTS_PER_REV
        );
    }

    @Override
    public void turnRobotToAngle(double target) {
        gyroLocked = true;
        gyroTarget = target;
        // Get the error. Positive means we need to rotate to the left
        double error = gyroTarget - imu.getZAngle();
        // Ensure we don't move farther than one rotation
        error %= 360;
        if(telemetry != null)
            telemetry.addData("Rot Error", error);
        // If we are within the requested tolerance (Constants.DRIVE_ANGLE_TOLERANCE), we should stop turning
        if(Math.abs(error) <= ANGLE_TOLERANCE) {
            stop();
            turningToAngle = false;
            return;
        } else {
            turningToAngle = true;
        }
        // As we approach the angle, we need to slow down the rotation
        double power = Range.clip(-error / 360, -0.5, 0.5);
        drive(power, power, -power, -power);
    }

    public void disableGyroLock(){
        gyroLocked = false;
    }

    public void setGyroLock(){
        gyroTarget = imu.getZAngle();
        gyroLocked = true;
    }

    public void turnToGyroLock() {
        turnRobotToAngle(gyroTarget);
    }
}