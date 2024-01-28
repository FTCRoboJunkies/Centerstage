package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualSlider2 {

    private static final double UP_POWER = 0.1;   // Power level when going up
    private static final double DOWN_POWER = -0.4; // Power level when going down

    private static double leftSliderPower;
    private static double rightSliderPower;

    private static int leftTarget = 0;
    private static int rightTarget = 0;

    private static DcMotor leftSliderMotor;
    private static DcMotor rightSliderMotor;

    private static int mainLeftTarget = 0;
    private static int mainRightTarget = 0;

    public enum LiftPosition {
        ZERO, LOW, MEDIUM, HIGH, AUTO_HIGH, AUTO_MEDIUM, AUTO_LOW, AUTO_GROUND, TRANSITION
    }

    public static void move(LiftPosition leftInput, LiftPosition rightInput) {
        move(getTargetPosition(leftInput), getTargetPosition(rightInput));
    }

    private static int getTargetPosition(LiftPosition input) {
        switch (input) {
            case ZERO:
                return 0;
            case LOW:
                return 1255;
            case MEDIUM:
                return 600;
            case HIGH:
                return 100;
            case AUTO_HIGH:
                return 1790;
            case AUTO_MEDIUM:
                return 1320;
            case AUTO_LOW:
                return 700;
            case AUTO_GROUND:
                return 100;
            case TRANSITION:
                return 500;
            default:
                return 0; // Default to 0 if the input is not recognized
        }
    }

    public static void move(int leftTarget, int rightTarget) {
        setLeftLiftPosition(leftTarget);
        setRightLiftPosition(rightTarget);
        mainLeftTarget = leftTarget;
        mainRightTarget = rightTarget;
    }

    public static void init(HardwareMap hardwareMap) {
        leftSliderMotor = hardwareMap.get(DcMotor.class, "LSliderMotor");
        rightSliderMotor = hardwareMap.get(DcMotor.class, "RSliderMotor");

        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private static void setLeftLiftPosition(int height) {
        leftTarget = height;
        double currentPos = leftSliderMotor.getCurrentPosition();

        if (currentPos < leftTarget) {
            // Going up
            leftSliderPower = UP_POWER;
        } else if (currentPos > leftTarget) {
            // Going down
            leftSliderPower = DOWN_POWER;
        } else {
            // Stop
            leftSliderPower = 0.0;
        }

        leftSliderMotor.setTargetPosition(leftTarget);
        leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSliderMotor.setPower(leftSliderPower);
    }

    private static void setRightLiftPosition(int height) {
        rightTarget = height;
        double currentPos = rightSliderMotor.getCurrentPosition();

        if (currentPos < rightTarget) {
            // Going up
            rightSliderPower = UP_POWER;
        } else if (currentPos > rightTarget) {
            // Going down
            rightSliderPower = DOWN_POWER;
        } else {
            // Stop
            rightSliderPower = 0.0;
        }

        rightSliderMotor.setTargetPosition(rightTarget);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSliderMotor.setPower(rightSliderPower);
    }

    public static int getLeftPosition() {
        return (leftSliderMotor.getCurrentPosition() / 2);
    }

    public static int getRightPosition() {
        return (rightSliderMotor.getCurrentPosition() / 2);
    }

    public void reset() {
        move(LiftPosition.ZERO, LiftPosition.ZERO);
    }

    public int getLeftTarget() {
        return mainLeftTarget;
    }

    public int getRightTarget() {
        return mainRightTarget;
    }
}
