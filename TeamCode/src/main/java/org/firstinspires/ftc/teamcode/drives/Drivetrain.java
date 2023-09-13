package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public abstract class Drivetrain {
    // Drivetrain constants
    public static final double ANGLE_TOLERANCE = 30; // The angle, in degrees, that is considered "close enough"
    public static final double MOTOR_MAX_SPEED = 0.3d;
    public static final double SWERVE_ENCODER_COUNTS_PER_REV = 2047.136; // Single revolution encoder ticks
    public static final double SWERVE_ENCODER_COUNTS_PER_INCH =  260.649; // Encoder Ticks per Inch
    public static final double SWERVE_WHEEL_ROT_MULTIPLIER = 1;
    public static final double SWERVE_FORWARD_SPEED_MULTIPLIER = 0.5;

    // Drivetrain instance-specific variables
    protected boolean isFieldCentric = true;
    protected Telemetry telemetry;
    protected IMU imu;

    // Drivetrain instance-specific variables
    protected boolean turningToAngle = false;

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
     * Non-blocking call to change motor power levels, limited by Drivetrain.MOTOR_MAX_SPEED
     * @param m1 motor power level 1
     * @param m2 motor power level 2
     * @param m3 motor power level 3
     * @param m4 motor power level 4
     */
    public abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * Blocking call to a robot rotation
    public abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * @param target the absolute angle to move to
     */
    public abstract void turnRobotToAngle(double target);

    /**
     * Blocking imu-adjusted field centric version of turnRobotToAngle
     * @param target The angle to adjust the current angle by
     */
    public void turnRobotByDegree(double target) { turnRobotToAngle(imu.getZAngle() + target); }

    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }
}
