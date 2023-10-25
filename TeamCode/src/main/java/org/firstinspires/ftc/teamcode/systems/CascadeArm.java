package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CascadeArm {
    public static final int MAXIMUM = 500;
    public static final int MINIMUM = 0;
    public static final double POWER = 0.5;
    protected DcMotor motor;
    protected Telemetry telemetry;

    public enum State {
        EXTENDED,
        CONTRACTED
    }
    public State state = State.CONTRACTED;

    public CascadeArm(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotor.class, "cascade");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;

    }
    public void moveArm(double strength){
        // raise arm OR lower arm
        int position = motor.getCurrentPosition();
        motor.setPower(POWER);
        state = State.EXTENDED;
        if((strength < 0 && position <= MAXIMUM) || (strength > 0 && position >= MINIMUM)) motor.setPower(strength * POWER);
        else motor.setPower(0);
    }


}