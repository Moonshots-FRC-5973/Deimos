package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class SwerveDrive extends Drivetrain {
    public final DcMotor leftMotorLeft;   // +Power = left wheel turning left
    public final DcMotor leftMotorRight;  // +Power = left wheel turning right
    public final DcMotor rightMotorLeft;  // +Power = right wheel turning right
    public final DcMotor rightMotorRight;
    private boolean fieldCentric = true;

    public int getLeftEncodersDifference() {
        return leftMotorLeft.getCurrentPosition() - leftMotorRight.getCurrentPosition();
    }
    public double getLeftWheelAngle() {
        double angle = (360 * (getLeftEncodersDifference() % ENCODER_COUNTS_PER_REV) / ENCODER_COUNTS_PER_REV);
        if(fieldCentric) return angle + getIMU().getZAngle();
        return angle;
    }
    public int getRightEncodersDifference() {
        return rightMotorLeft.getCurrentPosition() - rightMotorRight.getCurrentPosition();
    }
    public double getRightWheelAngle() {
        double angle = (360 * (getRightEncodersDifference() % ENCODER_COUNTS_PER_REV) / ENCODER_COUNTS_PER_REV);
        if (fieldCentric) return angle + getIMU().getZAngle();
        return angle;
    }
    public void makeRobotCentric() {
        fieldCentric = false;
    }
    public void switchMode() {
        fieldCentric = !fieldCentric;
    }
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public SwerveDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        leftMotorLeft = hardwareMap.get(DcMotor.class, "left_drive_left");
        leftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotorRight = hardwareMap.get(DcMotor.class, "left_drive_right");
        leftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorLeft = hardwareMap.get(DcMotor.class, "right_drive_left");
        rightMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorRight = hardwareMap.get(DcMotor.class, "right_drive_right");
        rightMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double strafe, double turn) {
        if(Math.abs(forward) <= Constants.INPUT_THRESHOLD) {
            forward = 0.0d;
        }
        if(Math.abs(strafe) <= Constants.INPUT_THRESHOLD) {
            strafe = 0.0d;
        }

        if(Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            drive(-turn, -turn, turn, turn);
            return;
        }

        // Translate XY input to degree and power
        double targetAngle = Math.atan2(forward, strafe); // -180 - 180 // Math.atan2 fails at -90 and 90
        double power = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));
        double leftAngleDiff = getLeftWheelAngle() - targetAngle;

        double ltPower = getLeftWheelAngle();

        // Quadrant Check to move in the correct direction / fastest direction
        if(leftAngleDiff <= 0 && leftAngleDiff > 90) {
            // Quadrant: 1
        } else if(leftAngleDiff <= 90 && leftAngleDiff > 180) {
            // Quadrant: 2
        } else if(leftAngleDiff <= 180 && leftAngleDiff > 270) {
            // Quadrant: 3
        } else {
            // Quadrant: 4
        }

        // Wheel | Motor
        // <l, r><l, r>Power
        double llPower = power;
        double lrPower = power;
        double rlPower = power;
        double rrPower = power;

        if(getTelemetry() != null) {// In case no messages should be sent.
            getTelemetry().addData("Target Angle", targetAngle);
            getTelemetry().addData("Power", power);
            getTelemetry().addData("Left Wheel Angle", getLeftWheelAngle());
            getTelemetry().addData("Right Wheel Angle", getRightWheelAngle());
        }

        drive(llPower, lrPower, rlPower, rrPower);
    }

    /**
     * @param m1 motor power level 1
     * @param m2 motor power level 2
     * @param m3 motor power level 3
     * @param m4 motor power level 4
     */
    @Override
    public void drive(double m1, double m2, double m3, double m4) {
        leftMotorLeft.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        leftMotorRight.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightMotorLeft.setPower(Range.clip(m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightMotorRight.setPower(Range.clip(m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    public void turnWheelToAngle(double target) {
        if(Math.abs(getLeftWheelAngle() - target) >= ANGLE_TOLERANCE) {
            leftMotorLeft.setPower(-0.75d);
            leftMotorRight.setPower(0.75d);
        }
        if(Math.abs(getRightWheelAngle() - target) >= ANGLE_TOLERANCE) {
            rightMotorLeft.setPower(-0.75d);
            rightMotorRight.setPower(0.75d);
        }
    }
    public void resetWheels() {
        drive(
                -leftMotorLeft.getCurrentPosition() / ENCODER_COUNTS_PER_REV,
                -leftMotorRight.getCurrentPosition() / ENCODER_COUNTS_PER_REV,
                -rightMotorLeft.getCurrentPosition() / ENCODER_COUNTS_PER_REV,
                -rightMotorRight.getCurrentPosition() / ENCODER_COUNTS_PER_REV
        );
    }

    @Override
    public void turnRobotToAngle(double target) {

    }
}