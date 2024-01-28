package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Consts.Launcher_Reload;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.Launcher_Launch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    public static Servo DroneServo;

    public static void init(HardwareMap hardwareMap) {
        DroneServo = hardwareMap.get(Servo.class, "DroneServo");
        move(Consts.Drone.Reload);
    }

    public static void move(Consts.Drone input) {
        // Servo limits go from 0 to 1
        switch (input) {
            case Reload:
                DroneServo.setPosition(Launcher_Reload);
                break;
            case Launch:
                DroneServo.setPosition(Launcher_Launch);
                break;
        }
    }

    public void moveLauncher(double target) {
        DroneServo.setPosition(target);
    }

    public void reset() {
        move(Consts.Drone.Reload);
    }

    public static double getPosition() {
        return DroneServo.getPosition();
    }
}

