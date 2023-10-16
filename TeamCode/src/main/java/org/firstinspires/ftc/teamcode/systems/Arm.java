package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public static final int UP_POSITION = 2000;
    public static final int DOWN_POSITION = 0;
    public static final double MOTOR_STRENGTH = 0.5;

    private Telemetry telemetry;
    private DcMotor motor;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.motor = hardwareMap.get(DcMotor.class, "arm");
        this.telemetry = telemetry;
        motor.setPower(MOTOR_STRENGTH);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void raise() {
        motor.setTargetPosition(UP_POSITION);
        motor.setPower(MOTOR_STRENGTH);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lower() {
        motor.setTargetPosition(DOWN_POSITION);
        motor.setPower(MOTOR_STRENGTH);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
