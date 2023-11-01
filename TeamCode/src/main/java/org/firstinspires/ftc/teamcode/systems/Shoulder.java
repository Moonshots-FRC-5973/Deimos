package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shoulder {
    public static final int UP_POSITION = 500;
    public static final int DOWN_POSITION = -1250;
    public static final double MOTOR_STRENGTH = 0.5;

    private Telemetry telemetry;
    private DcMotor motor;

    public Shoulder(HardwareMap hardwareMap, Telemetry telemetry) {
        this.motor = hardwareMap.get(DcMotor.class, "arm");
        this.telemetry = telemetry;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void move(double shoulderRot){
        if(telemetry != null) {
            telemetry.addData("Shoulder Pos", motor.getCurrentPosition());
            telemetry.addData("Shoulder Power", motor.getPower());
        }

        //shoulderRot *= -1;

        int position = motor.getCurrentPosition();
        if((shoulderRot > 0 && position <= UP_POSITION) || (shoulderRot < 0 && position >= DOWN_POSITION)) motor.setPower(MOTOR_STRENGTH * shoulderRot);
        else motor.setPower(0);
    }

}
