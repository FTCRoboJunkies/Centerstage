package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Consts.TILT_ANGLE_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.TILT_FRONT_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.TILT_BACK_LIMIT;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
   // public static Servo TLServo;
    public static Servo TRServo;


    public static void init(HardwareMap hardwareMap) {
       // TLServo = hardwareMap.get(Servo.class, "TLServo");
       /// move(Consts.Tilt.PICKUPTILT);
        TRServo = hardwareMap.get(Servo.class, "TRServo");
        move(Consts.Tilt.PICKUPTILT);
    }

    public static void move(Consts.Tilt input) {
        // Servo limits go from 0 to 1
        switch (input) {
            case SCORETILT:
                TRServo.setPosition(TILT_BACK_LIMIT);
              //  TLServo.setPosition(TILT_BACK_LIMIT);
                break;
            case PICKUPTILT:
                TRServo.setPosition(TILT_FRONT_LIMIT);
               // TLServo.setPosition(TILT_FRONT_LIMIT);
                break;
            case ANGLETILT:
                TRServo.setPosition(TILT_ANGLE_DOWN);
               // TLServo.setPosition(TILT_ANGLE_DOWN);
                break;
        }
    }

    public void moveTilt(double target) {
        TRServo.setPosition(target);
        //TLServo.setPosition(target);
    }

    public void reset() {
        move(Consts.Tilt.PICKUPTILT);
    }

    public static double getPosition() {

        TRServo.getPosition();
       // TLServo.getPosition();
        return TRServo.getPosition();



    }


}
