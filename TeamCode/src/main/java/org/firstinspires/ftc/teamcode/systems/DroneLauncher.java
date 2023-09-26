package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.HexMotor;

public class DroneLauncher {
    protected Telemetry telemetry;

    protected DcMotor motor1, motor2;

    protected double POWER = 1;

    public DroneLauncher (HardwareMap hardwareMap, Telemetry telemetry) {
        this.motor1 = hardwareMap.get(DcMotor.class, "Right Motor");
        this.motor2 = hardwareMap.get(DcMotor.class, "Left Motor");
        this.telemetry = telemetry;
    }

    public void lauchDrone() {
        motor2.setPower(-POWER);
        motor1.setPower(POWER);
    }
    public void stopDrone() {
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
