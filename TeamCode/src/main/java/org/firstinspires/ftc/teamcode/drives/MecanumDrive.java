package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive extends Drivetrain {

    // HARDWARE
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // GYRO TRACKERS
    private double gyroTarget;

    // BOOLEAN TOGGLES
    private boolean gyroLocked = false;
    private boolean useGyro = true;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, boolean gyroLocked) {
        super(hardwareMap, telemetry);

        this.useGyro = gyroLocked;
        // CONFIGURE HARDWARE
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor_drive");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_motor_drive");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor_drive");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_motor_drive");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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
            drive(turn, turn, -turn, -turn);
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
            telemetry.addData("Gyro Error", gyroError);
        }

        drive(
                forward + strafe + frontLeftBoost,
                forward - strafe + backLeftBoost,
                forward - strafe + frontRightBoost,
                forward + strafe + backRightBoost
        );

    }

    /**
     * Execute motion on the drivetrain
     * @param leftFront
     * @param leftRear
     * @param rightFront
     * @param rightRear
     */
    public void drive(double leftFront, double leftRear, double rightFront, double rightRear) {
        leftFrontDrive.setPower(Range.clip(leftFront, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(leftRear, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightFrontDrive.setPower(Range.clip(rightFront, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightBackDrive.setPower(Range.clip(rightRear, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    @Override
    public void turnRobotToAngle(double target) {

    }
}