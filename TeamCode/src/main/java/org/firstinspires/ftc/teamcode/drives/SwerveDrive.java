package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class SwerveDrive extends Drivetrain {
    public final DcMotor llMotor;   // +Power = left wheel turning left
    public final DcMotor lrMotor;  // +Power = left wheel turning right
    public final DcMotor rlMotor;  // +Power = right wheel turning right
    public final DcMotor rrMotor; // + Power = right wheel turning left

    /**
     * Returns the wheel angle
     * @param left The left motor for the wheel
     * @param right the right motor for the wheel
     * @return The angle of the wheel made up of the left and right motors
     */
    private double getWheelAngle(DcMotor left, DcMotor right) {
        double angle = 360 * ((left.getCurrentPosition() - right.getCurrentPosition()) % SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV;
        if(angle < 0) {
            angle += 360;
        }

        if(isFieldCentric) return angle + imu.getZAngle();
        return angle;
    }

    /**
     * <a href="https://www.geogebra.org/calculator/ucaxvmtw">See Here</a>
     * @param left The left motor for the wheel
     * @param right The right motor for the wheel
     * @param target The target angle the wheel should be pointing to
     * @return A proportional rotation power that describes the difference between
     * the left motor's power and the right motor's power
     */
    private double getWheelRotationPower(DcMotor left, DcMotor right, double target) {
        double wheelAngle = getWheelAngle(left, right);
        double angleDiff = target - wheelAngle;


        if (Math.abs(angleDiff) <= ANGLE_TOLERANCE) {
            return 0d;
        }

        // Rotate the opposite direction
        return Math.abs(Math.pow(Math.sin(Math.toRadians(angleDiff)), 1));
    }

    private double getWheelForwardMultiplier(DcMotor left, DcMotor right, double target) {
        double wheelAngle = getWheelAngle(left, right);
        double angleDiff = target - wheelAngle; // If it's on the other side it is not running since <= 180

        // Check both the near and supplementary angle
        if(Math.abs(angleDiff) <=  2 * ANGLE_TOLERANCE) {
            return SWERVE_FORWARD_SPEED_MULTIPLIER * Math.cos(Math.toDegrees(angleDiff));
        }
        return 0.0d;
    }

    public SwerveDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        llMotor = hardwareMap.get(DcMotor.class, "llMotor");
        lrMotor = hardwareMap.get(DcMotor.class, "lrMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");

        llMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lrMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rrMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        // Get information involving angles from the wheels and joystick
        double targetAngle = Math.toDegrees(Math.atan2(strafe, -forward)) + 180;
        double leftWheelDiff = targetAngle - getWheelAngle(llMotor, lrMotor);
        if (Math.abs(leftWheelDiff) <= ANGLE_TOLERANCE) leftWheelDiff = 0;
        double rightWheelDiff = targetAngle - getWheelAngle(rlMotor, rrMotor);
        if (Math.abs(rightWheelDiff) <= ANGLE_TOLERANCE) rightWheelDiff = 0;

        if(Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            drive(
                    (turn * getWheelForwardMultiplier(llMotor, lrMotor, 0)) + getWheelRotationPower(llMotor, lrMotor, 0),
                    (turn * getWheelForwardMultiplier(llMotor, lrMotor, 0)) - getWheelRotationPower(llMotor, lrMotor, 0),
                    (-turn * getWheelForwardMultiplier(rlMotor, rrMotor, 0)) + getWheelRotationPower(rlMotor, rrMotor, 0),
                    (-turn * getWheelForwardMultiplier(rlMotor, rrMotor, 0)) - getWheelRotationPower(rlMotor, rrMotor, 0)
            );
        }

        // Get the power of the motors
        double radius = Math.hypot(forward, strafe);
        double multiplier = (Math.abs(targetAngle - getWheelAngle(llMotor, lrMotor)) >= 90) ? -1 : 1;

        double llPower = radius * getWheelForwardMultiplier(llMotor, lrMotor, targetAngle);
        double lrPower = radius * getWheelForwardMultiplier(llMotor, lrMotor, targetAngle);
        double rlPower = radius * getWheelForwardMultiplier(rlMotor, rrMotor, targetAngle);
        double rrPower = radius * getWheelForwardMultiplier(rlMotor, rrMotor, targetAngle);

        llPower += getWheelRotationPower(llMotor, lrMotor, targetAngle);
        lrPower -= getWheelRotationPower(llMotor, lrMotor, targetAngle);
        rlPower += getWheelRotationPower(rlMotor, rlMotor, targetAngle);
        rrPower -= getWheelRotationPower(rlMotor, rrMotor, targetAngle);

        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        llPower *= multiplier;
        lrPower *= multiplier;
        rlPower *= multiplier;
        rrPower *= multiplier;

        telemetry.addData("Angle Diffs", "(%.2f, %.2f)", leftWheelDiff, rightWheelDiff);
        telemetry.addData("Motor Power", "(%.1f, %.1f, %.1f, %.1f)", llPower, lrPower, rlPower, rrPower);
        telemetry.addData("Encoders", "%d, %d, %d, %d)", llMotor.getCurrentPosition(), lrMotor.getCurrentPosition(), rlMotor.getCurrentPosition(), rrMotor.getCurrentPosition());

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
        double multiplier = (Math.abs(getWheelAngle(llMotor, lrMotor)) >= 90) ? -1 : 1;
        double leftRotPower = getWheelRotationPower(llMotor, lrMotor, target);
        double rightRotPower = getWheelRotationPower(rlMotor, rrMotor, target);
        double power = MOTOR_MAX_SPEED * multiplier;

        drive(
                power + leftRotPower,
                power - leftRotPower,
                -power + rightRotPower,
                -power - rightRotPower
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
    }
}
