package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public abstract class Drivetrain {
    // Drivetrain constants
    public static final double ANGLE_TOLERANCE = 1.5; // The angle, in degrees, that is considered "close enough"
    public static final double DISTANCE_TOLERANCE = 1.0; // The distance, in centimeters, that is considered "close enough"
    public static final double MOTOR_MAX_SPEED = 0.5;
    public static final double SWERVE_ENCODER_COUNTS_PER_REV = 2047.136; // Single revolution encoder ticks
    public static final double SWERVE_ENCODER_COUNTS_PER_INCH =  260.649; // Encoder Ticks per Inch
    public static final double SWERVE_WHEEL_ROT_MULTIPLIER = 3;

    // Drivetrain instance-specific variables
    protected boolean isFieldCentric = true;
    protected Telemetry telemetry;
    protected IMU imu;

    // Drivetrain instance-specific variables
    protected boolean turningToAngle = false;
    protected Telemetry telemetry;
    protected IMU imu;

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public IMU getIMU() {
        return imu;
    }

    public double speed = 1;
    protected boolean isFieldCentric = true;
    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }
    public boolean getFieldCentric() {
        return isFieldCentric;
    }

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, com.qualcomm.robotcore.hardware.IMU.Parameters parameters) {
        this.imu = new IMU(hardwareMap, parameters);
        this.telemetry = telemetry;
    }

    /**
     * @param forward a double to move in the forward direction
     * @param strafe a double to move in the horizontal direction
     * @param turn a double representing the speed to change the heading
     */
    public abstract void drive(double forward, double strafe, double turn);

    /**
     * @brief Non-blocking call to change motor power levels, limited by Drivetrain.MOTOR_MAX_SPEED
     * @param m1 motor power level 1
     * @param m2 motor power level 2
     * @param m3 motor power level 3
     * @param m4 motor power level 4
     */
    protected abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * @brief Blocking call to a robot rotation
    public abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * @param target the absolute angle to move to
     */
    public abstract void turnRobotToAngle(double target);

    /**
     * @brief Blocking imu-adjusted field centric version of turnRobotToAngle
     * @param target The angle to adjust the current angle by
     */
    public void turnRobotByDegree(double target) { turnRobotToAngle(imu.getZAngle() + target); }

    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }

    /**
     * @brief Blocking, IMU-stabilized call that adjusts the inputs to automatically run
     * @param angle the angle, relative to the field, that the robot needs to move in
     * @param distance the distance the robot needs to move on the specified angle
     */
    public void autoDriveInDirection(double angle, double distance) {
        double xDistance = Math.cos(angle) * distance;
        double yDistance = Math.sin(angle) * distance;

        while(xDistance > DISTANCE_TOLERANCE && yDistance > DISTANCE_TOLERANCE) {
            xDistance -= imu.getXVelocity();
            yDistance -= imu.getYVelocity();

            //drive(Range.clip(xDistance, -1, 1))
            return;
        }
        stop();
    }
}
