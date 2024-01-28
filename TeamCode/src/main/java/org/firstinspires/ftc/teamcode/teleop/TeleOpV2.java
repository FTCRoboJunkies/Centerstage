package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DualSlider;
import org.firstinspires.ftc.teamcode.subsystems.Lclaw;
import org.firstinspires.ftc.teamcode.subsystems.Rclaw;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

@TeleOp(name = "🟢Blur2")
public class TeleOpV2 extends LinearOpMode {

    Drivetrain drivetrain = new Drivetrain();
    Lclaw lclaw = new Lclaw();
    Rclaw rclaw = new Rclaw();
    Tilt tilt = new Tilt();

    DualSlider dualSlider = new DualSlider();

    @Override
    public void runOpMode() throws InterruptedException {
        Lclaw.init(hardwareMap);
        Rclaw.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        DualSlider.init(hardwareMap);
        Tilt.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            // Your existing teleop code here

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

            if (gamepad1.left_bumper) {
                Rclaw.move(Consts.Claw.ROPENCLAW);
                Lclaw.move(Consts.Claw.LOPENCLAW);
            } else if (gamepad1.right_bumper) {
                Lclaw.move(Consts.Claw.LCLOSECLAW);
                Rclaw.move(Consts.Claw.RCLOSECLAW);
            }
            if (gamepad2.x) {
                Tilt.move(Consts.Tilt.ANGLETILT); //tilt
            }
            if (gamepad2.a) {
                Tilt.move(Consts.Tilt.PICKUPTILT); //pickup
            } else if (gamepad2.y) {
                Tilt.move(Consts.Tilt.SCORETILT); //score
            }

            // Your other teleop logic here
        }
    }
}
