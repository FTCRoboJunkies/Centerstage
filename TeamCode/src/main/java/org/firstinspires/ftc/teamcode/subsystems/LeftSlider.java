package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LeftSlider {

    static double LSliderPower;

    public static int target=0;
    public boolean requestStop = false; //dead code, but used in deprecated teleoppromax so will not delete
    public static DcMotor LSliderMotor;
    public static int mainTarget=0;

    public static void move(Consts.Lift input) {
        switch (input) {
            case ZERO:
                move(0);
                break;


            case LOW:
                move(1255); // old: 780
                break;

            case MEDIUM:
                move(600); // old 1330
                break;

            case HIGH:
                move(100); // old 1870
                break;

            case AUTO_HIGH:
                move(1790);
                break;

            case AUTO_MEDIUM:
                move(1320);
                break;

            case AUTO_LOW:
                move(700);
                break;

            case AUTO_GROUND:
                move(100);
                break;
            case TRANSITION:
                move(500);
                break;
        }
    }

    public static void move(int target) {
        setLiftPosition(target);
        mainTarget = target;
    }

    public void init(HardwareMap hardwareMap) {
        LSliderMotor = hardwareMap.get(DcMotor.class, "LSliderMotor");
        LSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // LSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftSlider.move(Consts.Lift.ZERO);



        LSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public static void setLiftPosition(int height) {
        target = height;
        double currentPos = LSliderMotor.getCurrentPosition();

        if (currentPos < target) {
            // Going up
            LSliderPower = -0.1;

        } else if (currentPos > target) {
            // Going down
            LSliderPower = 0.4;

        }

        LSliderMotor.setTargetPosition(target);




        LSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LSliderMotor.setPower(LSliderPower);

    }

    public static int getPosition() {
        // Using the average of two left and right
        return (LSliderMotor.getCurrentPosition() / 2);
    }

    public void reset() {
        move(Consts.Lift.ZERO);
    }

    public int getTarget() {
        return mainTarget;
    }
}
