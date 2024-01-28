package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Consts.RCLAW_CLOSE_LIMIT;
        import static org.firstinspires.ftc.teamcode.subsystems.Consts.RCLAW_OPEN_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.LCLAW_CLOSE_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.LCLAW_OPEN_LIMIT;

        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;

public class Lclaw {


        public static Servo LCServo; //claw left


        public static void init(HardwareMap hardwareMap) {
            LCServo = hardwareMap.get(Servo.class, "CLServo");

            move(Consts.Claw.LCLOSECLAW);
        }

        public static void move(Consts.Claw input) {
            // Servo limits go from 0 to 1
            switch (input) {
                case LOPENCLAW:
                    LCServo.setPosition(LCLAW_OPEN_LIMIT);

                    break;
                case RCLOSECLAW:
                    LCServo.setPosition(LCLAW_CLOSE_LIMIT);

                    break;
            }
        }

        public void moveClaw(double target) {

            LCServo.setPosition(target);
        }

        public void reset() {
            move(Consts.Claw.LOPENCLAW);
        }

        public static double getPosition() {
            return LCServo.getPosition();
        }
    }


