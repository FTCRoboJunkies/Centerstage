package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RightSlider {

    static double RSliderPower;

    public static int target=0;
    public boolean requestStop = false; //dead code, but used in deprecated teleoppromax so will not delete
    public static DcMotor RSliderMotor;
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
        RSliderMotor = hardwareMap.get(DcMotor.class, "RSliderMotor");
        RSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightSlider.move(Consts.Lift.ZERO);



        RSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public static void setLiftPosition(int height) {
        target = height;
        double currentPos = RSliderMotor.getCurrentPosition();

        if (currentPos < target) {
            // Going up
            RSliderPower = 0.1;

        } else if (currentPos > target) {
            // Going down
            RSliderPower = -0.4;

        }

        RSliderMotor.setTargetPosition(target);




        RSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        RSliderMotor.setPower(RSliderPower);

    }

    public static int getPosition() {
        // Using the average of two left and right
        return (RSliderMotor.getCurrentPosition() / 2);
    }

    public void reset() {
        move(Consts.Lift.ZERO);
    }

    public int getTarget() {
        return mainTarget;
    }
}
