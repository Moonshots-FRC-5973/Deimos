package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constants;

public class IMU {
    public static final BNO055IMU.AngleUnit ANGLE_UNIT = BNO055IMU.AngleUnit.DEGREES;
    public static final BNO055IMU.AccelUnit ACCEL_UNIT = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    public static final String NAME = "imu";
    public static final String LOG_FILE_NAME = "imu.log";
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public IMU(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BNO055IMU.class, NAME);
        parameters.angleUnit = ANGLE_UNIT;
        parameters.accelUnit = ACCEL_UNIT;

        if(!LOG_FILE_NAME.equals("")) {
            parameters.loggingEnabled = true;
            parameters.calibrationDataFile = LOG_FILE_NAME;
        } else {
            parameters.loggingEnabled = false;
        }

        imu.initialize(parameters);
    }

    /**
     *
     * @return the X angle of the internal IMU in the control panel.
     */
    public double getXAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, ANGLE_UNIT.toAngleUnit()).firstAngle);
    }

    /**
     *
     * @return the Y angle of the internal IMU in the control panel.
     */
    public double getYAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, ANGLE_UNIT.toAngleUnit()).secondAngle);
    }

    /**
     *
     * @return the Z angle of the internal IMU in the control panel.
     */
    public double getZAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, ANGLE_UNIT.toAngleUnit()).thirdAngle);
    }

    /**
     *
     * @return a double array, ordered XYZ, of the angle.
     */
    public double[] getAngle() {
        double[] out = new double[3];

        out[0] = getXAngle();
        out[1] = getYAngle();
        out[2] = getZAngle();

        return out;
    }

    /**
     *
     * @return The X axis velocity of the control panel.
     */
    public double getXVelocity() {
        return imu.getVelocity().xVeloc;
    }

    /**
     *
     * @return The Y axis velocity of the control panel.
     */
    public double getYVelocity() {
        return imu.getVelocity().yVeloc;
    }

    /**
     *
     * @return The Z axis velocity of the control panel.
     */
    public double getZVelocity() {
        return imu.getVelocity().zVeloc;
    }

    /**
     *
     * @return An ordered XYZ array of the control panel's current velocity.
     */
    public double[] getVelocity() {
        double[] out = new double[3];

        out[0] = getXVelocity();
        out[1] = getYVelocity();
        out[2] = getZVelocity();

        return out;
    }

    public double getXPosition() {
        return imu.getPosition().x;
    }

    public double getYPosition() {
        return imu.getPosition().y;
    }

    public double getZPosition() {
        return imu.getPosition().z;
    }

    public double[] getPosition() {
        double[] out = new double[3];

        out[0] = getXPosition();
        out[1] = getYPosition();
        out[2] = getZPosition();

        return out;
    }
}
