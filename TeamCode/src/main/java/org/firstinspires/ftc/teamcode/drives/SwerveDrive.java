package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class SwerveDrive extends Drivetrain {
    // Left side, left motor
    public final DcMotor llMotor;
    // Left side, right motor
    public final DcMotor lrMotor;
    // Right side, left motor
    public final DcMotor rlMotor;
    // Right side, right motor
    public final DcMotor rrMotor;
    // Autonomous control functions
    private boolean moveForwardAuto = false;
    private boolean rotateRobotAuto = false;
    private boolean rotateWheelAuto = false;
    private double targetWheelAngle = 0.0d;
    private double targetRobotAngle = 0.0d;


    public SwerveDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        llMotor = hardwareMap.get(DcMotor.class, "llMotor");
        lrMotor = hardwareMap.get(DcMotor.class, "lrMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        // Check auto booleans, when running the auto method, the Autonomous program is expected to take into account no field centric
        if(rotateRobotAuto) {
            turnRobotToAngle(targetRobotAngle);
        } else if(rotateWheelAuto) {
            turnWheelToAngle(targetWheelAngle);
        }

        // Useful Variables for later
        double rightWheelAngle = (360 * ((rlMotor.getCurrentPosition() - rrMotor.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV);
        double leftWheelAngle = (360 * ((llMotor.getCurrentPosition() - lrMotor.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV);
        double leftRotPower, rightRotPower;
        double multiplier = 1;

        // ______________
        // -- ROTATION --
        if(Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            // If Wheel's forward face is away from the 0 mark, reverse rotation
            if(Math.abs(leftWheelAngle) >= 90) {
                multiplier = -1;
            }
            // multiplier flips rotation direction; Rest calculates how much rotation we need
            // https://www.geogebra.org/calculator/ucaxvmtw
            leftRotPower = SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-leftWheelAngle));
            rightRotPower = SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-rightWheelAngle));

            drive(
                    multiplier * (-turn + leftRotPower),
                    multiplier * (-turn - leftRotPower),
                    multiplier * (rightRotPower - turn),
                    multiplier * (-turn - rightRotPower));
            return;
        }

        // Field Centric adjustment
        if (isFieldCentric) {
            // Learn more:
            // https://www.geogebra.org/m/fmegkksm
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(imu.getZAngle())) - strafe * Math.sin(Math.toRadians(imu.getZAngle()));
            strafe = temp * Math.sin(Math.toRadians(imu.getZAngle())) + strafe * Math.cos(Math.toRadians(imu.getZAngle()));
        }

        // Calculate magnitude of joystick being pushed
        double radius = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));

        if(radius <= Constants.INPUT_THRESHOLD) {
            if (!moveForwardAuto && !rotateWheelAuto) {
                stop();
            } else if(rotateRobotAuto) {
                turnRobotToAngle(targetRobotAngle);
            } else if(rotateWheelAuto) {
                turnWheelToAngle(targetWheelAngle);
            }
            return;
        }

        moveForwardAuto = false;
        rotateWheelAuto = false;
        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Left is reversed due to gearing
        double llPower = -radius;
        double lrPower = -radius;
        double rlPower = radius;
        double rrPower = radius;

        // Point wheels in correct direction
        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward));
        double leftAngleDiff = targetAngle - leftWheelAngle;
        double rightAngleDiff = targetAngle - rightWheelAngle;

        if(telemetry != null) {
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Left Wheel Diff", leftAngleDiff);
            telemetry.addData("Right Wheel Diff", rightAngleDiff);
        }

        // Wheel Dead Zones
        if(Math.abs(leftAngleDiff) <= ANGLE_TOLERANCE) {
            leftAngleDiff = 0.0d;
        }
        if(Math.abs(rightAngleDiff) <= ANGLE_TOLERANCE) {
            rightAngleDiff = 0.0d;
        }

        // Reverse power if closer to go to 180 than 0
        if (Math.abs(leftAngleDiff) >= 90){
            multiplier = -1;
        }

        llPower *= multiplier;
        lrPower *= multiplier;
        rlPower *= multiplier;
        rrPower *= multiplier;

        leftRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(leftAngleDiff));
        llPower += leftRotPower;
        lrPower -= leftRotPower;
        rightRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(rightAngleDiff));
        rlPower += rightRotPower;
        rrPower -= rightRotPower;

        // Add Forward Drive power
        drive(llPower, lrPower, rlPower, rrPower);
    }

    @Override
    public void drive(double m1, double m2, double m3, double m4) {
        llMotor.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        lrMotor.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rlMotor.setPower(Range.clip(m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rrMotor.setPower(Range.clip(m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    @Override
    public void turnRobotToAngle(double target) {
        double rightWheelAngle = (360 * ((rlMotor.getCurrentPosition() - rrMotor.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV);
        double leftWheelAngle = (360 * ((llMotor.getCurrentPosition() - lrMotor.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV);
        double multiplier = 1;
        double leftRotPower, rightRotPower;
        // If Wheel's forward face is away from the 0 mark, reverse rotation
        if(Math.abs(leftWheelAngle) >= 90) {
            multiplier = -1;
        }
        // multiplier flips rotation direction; Rest calculates how much rotation we need
        // https://www.geogebra.org/calculator/ucaxvmtw
        leftRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-leftWheelAngle));
        rightRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-rightWheelAngle));
        drive(
                -(MOTOR_MAX_SPEED * multiplier) + leftRotPower,
                -(MOTOR_MAX_SPEED * multiplier) - leftRotPower,
                rightRotPower - (MOTOR_MAX_SPEED * multiplier),
                -(MOTOR_MAX_SPEED * multiplier) - rightRotPower
        );
    }

    public void setDistanceToTravel(double distance) {
        llMotor.setTargetPosition(
                llMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );
        lrMotor.setTargetPosition(
                lrMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );
        rlMotor.setTargetPosition(
                rlMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );
        rrMotor.setTargetPosition(
                rrMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );
        llMotor.setPower(MOTOR_MAX_SPEED);
        lrMotor.setPower(MOTOR_MAX_SPEED);
        rlMotor.setPower(MOTOR_MAX_SPEED);
        rrMotor.setPower(MOTOR_MAX_SPEED);
        llMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveForwardAuto = true;
    }

    public void turnWheelToAngle(double angle) {
        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.targetWheelAngle = angle;
        this.rotateWheelAuto = true;
        double rightWheelAngle = (360 * ((rlMotor.getCurrentPosition() - rrMotor.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV);
        double leftWheelAngle = (360 * ((llMotor.getCurrentPosition() - lrMotor.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV);
        double multiplier = 1;
        double leftRotPower, rightRotPower;
        if(Math.abs(angle - rightWheelAngle) <= ANGLE_TOLERANCE) {
            this.rotateWheelAuto = false;
            stop();
            return;
        }

            // If Wheel's forward face is away from the 0 mark, reverse rotation
        if(Math.abs(angle - leftWheelAngle) >= 90) {
            multiplier = -1;
        }
        // multiplier flips rotation direction; Rest calculates how much rotation we need
        // https://www.geogebra.org/calculator/ucaxvmtw
        leftRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(angle - leftWheelAngle));
        rightRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(angle - rightWheelAngle));
        drive(leftRotPower, -leftRotPower, rightRotPower, -rightRotPower);
    }
}