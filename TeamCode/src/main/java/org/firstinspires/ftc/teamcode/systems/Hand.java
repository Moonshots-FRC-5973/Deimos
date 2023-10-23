package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hand {
    private final double OPEN_POS = 1;
    private final double CLOSED_POS = 0;
    private final double WRIST_INC = 1;
    private final double WRIST_MAX = 0;
    private final double WRIST_MIN = 0;

    private double wristAng = 90;

    private Servo wristServo;
    private Servo firstSlotServo;
    private Servo secondSlotServo;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public enum State {
        OPEN,
        CLOSED
    }
    public Hand.State state = State.CLOSED;

    public Hand(HardwareMap hardwareMap, Telemetry telemetry) {
        wristServo = hardwareMap.get(Servo.class, "wrist");
        firstSlotServo = hardwareMap.get(Servo.class, "slot1");
        secondSlotServo = hardwareMap.get(Servo.class, "slot2");
        this.telemetry = telemetry;
    }

    public void openBoth() {
        firstSlotServo.setPosition(OPEN_POS);
        secondSlotServo.setPosition(OPEN_POS);
        state = State.OPEN;
    }

    public void closeBoth() {
        firstSlotServo.setPosition(CLOSED_POS);
        secondSlotServo.setPosition(CLOSED_POS);
        state = State.CLOSED;
    }

    public void wristUp() {
        wristAng += WRIST_INC;
        if (wristAng >= WRIST_MAX ) {
            wristAng = WRIST_MAX;
        }
        wristServo.setPosition(wristAng);
    }

    public void wristDown() {
        wristAng -= WRIST_INC;
        if (wristAng <= WRIST_MIN ) {
            wristAng = WRIST_MIN;
        }
        wristServo.setPosition(wristAng);
    }
}
