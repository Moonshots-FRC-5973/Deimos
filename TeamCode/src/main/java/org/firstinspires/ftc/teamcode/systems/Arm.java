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
    private CascadeArm cascadeArm;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.motor = hardwareMap.get(DcMotor.class, "arm");
        this.telemetry = telemetry;
        this.cascadeArm = new CascadeArm(hardwareMap,telemetry);

    }

    public void move(double shoulderRot, double armStrength){
        int position = motor.getCurrentPosition();
        motor.setPower(shoulderRot);
        if((shoulderRot < 0 && position <= UP_POSITION) || (shoulderRot > 0 && position >= DOWN_POSITION)) motor.setPower(shoulderRot);
        else motor.setPower(0);
        cascadeArm.moveArm(armStrength);
    }

}
