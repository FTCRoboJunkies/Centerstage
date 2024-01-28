package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Consts.LCLAW_CLOSE_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.LCLAW_OPEN_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.RCLAW_CLOSE_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.RCLAW_OPEN_LIMIT;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Rclaw {


        public static Servo CRServo; //claw left


        public static void init(HardwareMap hardwareMap) {
            CRServo = hardwareMap.get(Servo.class, "CRServo");

            move(Consts.Claw.RCLOSECLAW);
        }

        public static void move(Consts.Claw input) {
            // Servo limits go from 0 to 1
            switch (input) {
                case ROPENCLAW:
                    CRServo.setPosition(RCLAW_OPEN_LIMIT);

                    break;
                case RCLOSECLAW:
                    CRServo.setPosition(RCLAW_CLOSE_LIMIT);

                    break;
            }
        }

        public void moveClaw(double target) {

            CRServo.setPosition(target);
        }

        public void reset() {
            move(Consts.Claw.ROPENCLAW);
        }

        public static double getPosition() {
            return CRServo.getPosition();
        }
    }


