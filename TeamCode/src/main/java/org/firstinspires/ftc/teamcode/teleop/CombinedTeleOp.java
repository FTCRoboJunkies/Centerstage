package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DualSlider;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem; // Import the updated ClawSubsystem

@TeleOp(name = "🟢CombinedTeleOp")
@Config()
//@Disabled
public class CombinedTeleOp extends LinearOpMode {

    // Define constants for motor power adjustment
    private static final double NORMAL_POWER = 1.0;
    private static final double SLOW_POWER = 0.5;

    // Track whether slow mode is active
    private boolean slowMode = false;

    Drivetrain drivetrain = new Drivetrain();
    DualSlider dualSlider = new DualSlider();
    Tilt tilt = new Tilt();
    ClawSubsystem clawSubsystem = new ClawSubsystem(); // Initialize the ClawSubsystem

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        DualSlider.init(hardwareMap);
        Tilt.init(hardwareMap);
        clawSubsystem.init(hardwareMap); // Initialize the ClawSubsystem

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            // Control Drivetrain
            double vertical = -gamepad1.right_stick_y * powerMultiplier;
            double horizontal = gamepad1.right_stick_x * powerMultiplier;
            double rotation = gamepad1.left_stick_x * powerMultiplier;

            // Call power() method with individual powers for each motor
            Drivetrain.power(vertical - horizontal + rotation,
                    vertical + horizontal + rotation,
                    vertical + horizontal - rotation,
                    vertical - horizontal - rotation);

            // Example: Move both sliders to HIGH position when dpad_up is pressed
            if (gamepad2.dpad_up) {
                DualSlider.move(DualSlider.LiftPosition.HIGH, DualSlider.LiftPosition.HIGH);
            } else if (gamepad2.dpad_right) {
                DualSlider.move(DualSlider.LiftPosition.LOW, DualSlider.LiftPosition.LOW);
            } else if (gamepad2.dpad_left) {
                DualSlider.move(DualSlider.LiftPosition.MEDIUM, DualSlider.LiftPosition.MEDIUM);
            } else if (gamepad2.dpad_down) {
                DualSlider.move(DualSlider.LiftPosition.ZERO, DualSlider.LiftPosition.ZERO);
            }

            // Control claws
            if (gamepad1.left_bumper) {
                clawSubsystem.moveLeftClaw(Consts.LCLAW_OPEN_LIMIT);
                clawSubsystem.moveRightClaw(Consts.RCLAW_OPEN_LIMIT);
            } else if (gamepad1.right_bumper) {
                clawSubsystem.moveLeftClaw(Consts.LCLAW_CLOSE_LIMIT);
                clawSubsystem.moveRightClaw(Consts.RCLAW_CLOSE_LIMIT);
            }

            // Control tilt
            if (gamepad2.x) {
                Tilt.move(Consts.Tilt.ANGLETILT); //tilt
            }
            if (gamepad2.a) {
                Tilt.move(Consts.Tilt.PICKUPTILT); //pickup
            } else if (gamepad2.y) {
                Tilt.move(Consts.Tilt.SCORETILT); //score
            }

            // Rest of your teleop logic

            // Example: Control additional mechanisms or features

            // if (gamepad1.a) {
            //     // Do something when gamepad1's A button is pressed
            // }

            // if (gamepad2.left_trigger > 0.5) {
            //     // Do something when gamepad2's left trigger is pressed halfway
            // }
        }
    }
}
