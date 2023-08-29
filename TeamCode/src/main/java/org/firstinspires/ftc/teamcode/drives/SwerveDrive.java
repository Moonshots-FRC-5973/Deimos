package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        double angle = 360 * ((left.getCurrentPosition() - right.getCurrentPosition()) / SWERVE_ENCODER_COUNTS_PER_REV ) % SWERVE_ENCODER_COUNTS_PER_REV;
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
        double multiplier = (Math.abs(wheelAngle) >= 90) ? -1 : 1;
        double angleDiff = target - wheelAngle;

        if (angleDiff >= ANGLE_TOLERANCE) {
            return 0d;
        }

        return multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(angleDiff));
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
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        // Point wheels in correct direction
        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward));
        double leftRotPower = getWheelRotationPower(llMotor, lrMotor, targetAngle);
        double rightRotPower = getWheelRotationPower(rlMotor, rrMotor, targetAngle);
        double multiplier = (Math.abs(getWheelAngle(llMotor, lrMotor)) >= 90) ? -1 : 1;

        // ______________
        // -- ROTATION --
        if(Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            drive(
                    leftRotPower + (multiplier * turn),
                    -leftRotPower + (multiplier * turn),
                    rightRotPower - (multiplier * turn),
                    -rightRotPower - (multiplier * turn));
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

        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Left is reversed due to gearing
        double llPower = -radius;
        double lrPower = -radius;
        double rlPower = radius;
        double rrPower = radius;


        if(telemetry != null) {
            telemetry.addData("Target Angle", targetAngle);
        }

        llPower *= multiplier;
        lrPower *= multiplier;
        rlPower *= multiplier;
        rrPower *= multiplier;

        llPower += leftRotPower;
        lrPower -= leftRotPower;

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
