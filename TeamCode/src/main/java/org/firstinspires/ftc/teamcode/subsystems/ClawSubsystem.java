package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {
    private Servo leftClawServo;
    private Servo rightClawServo;

    private static final double LEFT_OPEN_LIMIT = 0.53;
    private static final double LEFT_CLOSE_LIMIT = 0.5;
    private static final double RIGHT_OPEN_LIMIT = 0.43;
    private static final double RIGHT_CLOSE_LIMIT = 0.5;

    public void init(HardwareMap hardwareMap) {
        leftClawServo = hardwareMap.get(Servo.class, "CLServo");
        rightClawServo = hardwareMap.get(Servo.class, "CRServo");

        // Initialize claws to closed position
        leftClawServo.setPosition(LEFT_CLOSE_LIMIT);
        rightClawServo.setPosition(RIGHT_CLOSE_LIMIT);
    }

    public void moveLeftClaw(double position) {
        leftClawServo.setPosition(position);
    }

    public void moveRightClaw(double position) {
        rightClawServo.setPosition(position);
    }

    public void closeClaws() {
        leftClawServo.setPosition(LEFT_CLOSE_LIMIT);
        rightClawServo.setPosition(RIGHT_CLOSE_LIMIT);
    }

    public void openClaws() {
        leftClawServo.setPosition(LEFT_OPEN_LIMIT);
        rightClawServo.setPosition(RIGHT_OPEN_LIMIT);
    }

    public double getLeftClawPosition() {
        return leftClawServo.getPosition();
    }

    public double getRightClawPosition() {
        return rightClawServo.getPosition();
    }
}
