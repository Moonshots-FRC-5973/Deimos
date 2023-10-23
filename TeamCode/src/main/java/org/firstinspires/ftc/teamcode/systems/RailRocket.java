package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RailRocket {
    protected Telemetry telemetry;

    protected DcMotor motor1, motor2;

    /*
    There should be a code to make sure that the servo is pulling way (to take the rod out) on
    comand + hold the rilingsand place corpse in place .
    Servo do something
    This file will be all coded by Ed + Stacy. Last modified 10/20/2023 11:54am worked on comments and basically just started
     */
    protected double POWER = 1;

    public RailRocket (HardwareMap hardwareMap, Telemetry telemetry) {
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
