package org.firstinspires.ftc.teamcode.drives;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class NewSwerveDrive extends Drivetrain {
    private DcMotor llMotor, lrMotor, rlMotor, rrMotor;


    public NewSwerveDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, null);
        llMotor = hardwareMap.get(DcMotor.class, "swerveLLMotor");
        lrMotor = hardwareMap.get(DcMotor.class, "swerveLRMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "swerveRLMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "swerveRRMotor");
    }

    private double getWheelAngle(@NonNull DcMotor lMotor, @NonNull DcMotor rMotor) {
        return 360 * ((lMotor.getCurrentPosition() - rMotor.getCurrentPosition()) %
                SWERVE_ENCODER_COUNTS_PER_REV) / SWERVE_ENCODER_COUNTS_PER_REV;
    }

    private void rotateRobot(double turn) {
        double leftWheelAngle = getWheelAngle(llMotor, lrMotor);
        double rightWheelAngle = getWheelAngle(rlMotor, rrMotor);
        double multiplier = 1;
        // If Wheel's forward face is away from the 0 mark, reverse rotation
        if(Math.abs(leftWheelAngle) >= 90) {
            multiplier = -1;
        }
        // multiplier flips rotation direction; Rest calculates how much rotation we need
        // https://www.geogebra.org/calculator/ucaxvmtw
        double leftRotPower = SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-leftWheelAngle));
        double rightRotPower = SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-rightWheelAngle));

        drive(
                multiplier * (-turn + leftRotPower),
                multiplier * (-turn - leftRotPower),
                multiplier * (rightRotPower - turn),
                multiplier * (-turn - rightRotPower));
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        // check robot rotation
        if(turn >= Constants.INPUT_THRESHOLD) {
            if(telemetry != null) {
                telemetry.addData("Drive Mode", "Robot Rotation");
            }
            rotateRobot(turn);
        }

        // check field centric rotation
        if(isFieldCentric) {
            if(telemetry != null) {
                telemetry.addData("Drive Mode", "Field Centric Driving");
            }
            // Learn more:
            // https://www.geogebra.org/m/fmegkksm
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(imu.getZAngle())) - strafe *
                    Math.sin(Math.toRadians(imu.getZAngle()));
            strafe = temp * Math.sin(Math.toRadians(imu.getZAngle())) + strafe *
                    Math.cos(Math.toRadians(imu.getZAngle()));
        } else {
            if(telemetry != null) {
                telemetry.addData("Drive Mode", "Robot Centric Driving");
            }
        }

        // Convert to radial graph
        double radius = Math.hypot(forward, strafe);

        // Get Wheel Angles
        double rightWheelAngle = getWheelAngle(rlMotor, rrMotor);
        double leftWheelAngle = getWheelAngle(llMotor, lrMotor);

        // Find Difference from target
        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward));
        double leftAngleDiff = targetAngle - leftWheelAngle;
        double rightAngleDiff = targetAngle - rightWheelAngle;

        // Wheel Dead Zones
        if(Math.abs(leftAngleDiff) <= ANGLE_TOLERANCE) { leftAngleDiff = 0.0d; }
        if(Math.abs(rightAngleDiff) <= ANGLE_TOLERANCE) { rightAngleDiff = 0.0d; }

        // telemetry outputs
        if(telemetry != null) {
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Left Wheel Diff", leftAngleDiff);
            telemetry.addData("Right Wheel Diff", rightAngleDiff);
        }

        // Left is reversed due to gearing
        double llPower = -radius;
        double lrPower = -radius;
        double rlPower = radius;
        double rrPower = radius;

        double multiplier = 1;
        // Reverse power if closer to go to 180 than 0
        if (Math.abs(leftAngleDiff) >= 90){
            multiplier = -1;
        }

        llPower *= multiplier;
        lrPower *= multiplier;
        rlPower *= multiplier;
        rrPower *= multiplier;

        double leftRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(leftAngleDiff));
        double rightRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(rightAngleDiff));
        llPower += leftRotPower;
        lrPower -= leftRotPower;
        rlPower += rightRotPower;
        rrPower -= rightRotPower;

        drive(llPower, lrPower, rlPower, rrPower);
    }

    @Override
    public void drive(double m1, double m2, double m3, double m4) {
        // Ensure Motor Limits are set properly
        llMotor.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        lrMotor.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rlMotor.setPower(Range.clip(m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rrMotor.setPower(Range.clip(m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    @Override
    public void turnRobotToAngle(double target) {

    }
}
