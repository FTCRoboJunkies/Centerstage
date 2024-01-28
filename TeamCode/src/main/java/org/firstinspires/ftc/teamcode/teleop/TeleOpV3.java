package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "🟢Blur3")
@Config()
//@Disabled
public class TeleOpV3 extends LinearOpMode {

    // Define constants for motor power adjustment
    private static final double NORMAL_POWER = 1.0;
    private static final double SLOW_POWER = 0.5;

    // Track whether slow mode is active
    private boolean slowMode = false;

    Drivetrain drivetrain = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            // Check for slow mode activation
            if (gamepad1.a) {
                slowMode = true;
            }

            // Check for returning to normal mode
            if (gamepad1.b) {
                slowMode = false;
            }

            // Adjust motor powers based on slow mode
            double powerMultiplier = slowMode ? SLOW_POWER : NORMAL_POWER;

            // Your motor control code here, adjusting powers using powerMultiplier
            double vertical = -gamepad1.right_stick_y * powerMultiplier;
            double horizontal = gamepad1.right_stick_x * powerMultiplier;
            double rotation = gamepad1.left_stick_x * powerMultiplier;

            // Call the Drivetrain's power method with individual powers for each motor
            drivetrain.power(vertical - horizontal + rotation,
                    vertical + horizontal + rotation,
                    vertical + horizontal - rotation,
                    vertical - horizontal - rotation);

            // Rest of your teleop logic
        }
    }
}
