package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ArcadeDrive extends Drivetrain {
    // Motors for the drivetrain
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // Target angle for turning the robot to
    private double targetAngle;
    /**
     * Constructor for the ArcadeDrive class. It takes in a HardwareMap, ElapsedTime, and Telemetry
     * object, and passes them to the superclass constructor. It also initializes the left and right
     * drive motors by getting the appropriate hardware from the HardwareMap.
     *
     * @param hardwareMap HardwareMap object containing the hardware configuration of the robot
     * @param telemetry Telemetry object to allow communication with the driver station
     */
    public ArcadeDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
    }

    /**
     * This method drives the robot with arcade controls, using a forward and strafe input, and a turn
     * input. It converts these inputs to individual motor powers, and calls the drive() method with
     * the calculated motor powers.
     *
     * @param forward Forward input, ranging from -1.0 to 1.0
     * @param strafe Strafe input, ranging from -1.0 to 1.0
     * @param turn Turn input, ranging from -1.0 to 1.0
     */
    @Override
    public void drive(double forward, double strafe, double turn) {
        drive(forward + turn, forward - turn, 0.0d, 0.0d);
    }

    /**
     * This method sets the powers of the individual drive motors. It takes in four motor powers as
     * arguments, and sets the left and right drive motors to the appropriate powers. It also clips
     * the power values to the maximum motor speed defined in Constants.
     *
     * @param m1 Power for the first motor, ranging from -1.0 to 1.0
     * @param m2 Power for the second motor, ranging from -1.0 to 1.0
     * @param m3 Power for the third motor, ranging from -1.0 to 1.0
     * @param m4 Power for the fourth motor, ranging from -1.0 to 1.0
     */
    @Override
    public void drive(double m1, double m2, double m3, double m4) {
        leftDrive.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightDrive.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }

    @Override
    public void resetWheels() {
        drive(-imu.getZAngle() / 100, imu.getZAngle() / 100, 0.0d, 0.0d);
    }

    @Override
    public void turnRobotToAngle(double target) {

    }
}
