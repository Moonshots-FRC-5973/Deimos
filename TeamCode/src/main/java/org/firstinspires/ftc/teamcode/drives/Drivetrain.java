package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public abstract class Drivetrain {
    // Drivetrain constants
    public static final double ANGLE_TOLERANCE = 1.5; // The angle, in degrees, that is considered "close enough"
    public static final double ENCODER_COUNTS_PER_REV = 1023.568; // Single revolution encoder ticks
    public static final double MOTOR_MAX_SPEED = 0.3;

    // Drivetrain instance-specific variables
    protected boolean turningToAngle = false;

    // General Drivetrain subsystems and get methods for them
    protected Telemetry telemetry;
    public Telemetry getTelemetry() {
        return telemetry;
    }

    protected IMU imu;
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
    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.imu = new IMU(hardwareMap);
        this.telemetry = telemetry;
    }

    /**
     * @param forward a double to move in the forward direction
     * @param strafe a double to move in the horizontal direction
     * @param turn a double representing the speed to change the heading
     */
    public abstract void drive(double forward, double strafe, double turn);

    /**
     * @param m1 motor power level 1
     * @param m2 motor power level 2
     * @param m3 motor power level 3
     * @param m4 motor power level 4
     */
    public abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * DEPRECEATED: There is no reason to use this
     */
    @Deprecated
    public abstract void resetWheels();

    /**
     * @param target the absolute angle to move to
     */
    public abstract void turnRobotToAngle(double target);

    /**
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
