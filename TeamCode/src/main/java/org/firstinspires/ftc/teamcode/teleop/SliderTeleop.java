package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DualSlider2;
import org.firstinspires.ftc.teamcode.subsystems.DualSlider2.LiftPosition;

@TeleOp(name="SliderTeleop", group="TeleOp")
public class SliderTeleop extends LinearOpMode {

    // Declare the DualSlider2 subsystem
    private DualSlider2 dualSlider2;

    @Override
    public void runOpMode() {
        // Initialize the DualSlider2 subsystem
        dualSlider2 = new DualSlider2();
        dualSlider2.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Use D-pad on gamepad 2 to control slider positions
            if (gamepad2.dpad_up) {
                dualSlider2.move(LiftPosition.LOW, LiftPosition.LOW);
            } else if (gamepad2.dpad_down) {
                dualSlider2.move(LiftPosition.ZERO, LiftPosition.ZERO);
            }

            if (gamepad2.dpad_left) {
                dualSlider2.move(LiftPosition.MEDIUM, LiftPosition.ZERO);
            } else if (gamepad2.dpad_right) {
                dualSlider2.move(LiftPosition.ZERO, LiftPosition.MEDIUM);
            }

            // Display telemetry
            telemetry.addData("Left Slider Position", dualSlider2.getLeftPosition());
            telemetry.addData("Right Slider Position", dualSlider2.getRightPosition());
            telemetry.update();
        }
    }
}
