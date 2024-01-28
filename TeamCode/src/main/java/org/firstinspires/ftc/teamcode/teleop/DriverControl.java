package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareTeleop;
//import org.firstinspires.ftc.teamcode.subsystems.Tilt;


/*
 * Key Map
 * Gamepad 1
 * Left joystick: Translation
 * DPad: Slow translation
 * Right joystick: Rotation
 * Bumpers: Slow rotation
 * -----------------------
 * Gamepad 2
 * A: Toggle Claw
 * B: Toggle Belt
 * DPad up: Move lift to high
 * DPad left: Move lift to mid
 * DPad right: Move lift to low
 * DPad down: Move lift to zero
 * Right joystick: Turntable rotation
 */
@TeleOp(name="Mecanum: Driver Control", group="Mecanum")
//@Config
//@Disabled
public class DriverControl extends OpMode {
    // Declare OpMode members.
    HardwareTeleop robot = new HardwareTeleop(); // Use a Mecanum's hardware


    // boolean clawOpen = false;


    // Claw claw = new Claw();
    //        OldBelt oldBelt = new OldBelt();

    //Lift lift = new Lift();

    //Intake intake = new Intake();
    //Drivetrain drivetrain = new Drivetrain();


    // boolean gp2AReleased = true;

    // boolean currentAbtn;

    @Override
    public void init() {

        //Initialize the hardware variables
        robot.init(hardwareMap);


        //claw.init(hardwareMap);

        // lift.init(hardwareMap);

        //intake.init(hardwareMap);

        //Drivetrain.init(hardwareMap);


        //Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //boolean clawOpen = false;


        //  Intake intake = new Intake();
        //  Claw claw = new Claw();
        // Lift lift = new Lift();
        //claw.init(hardwareMap);
        //intake.init(hardwareMap);
        // lift.init(hardwareMap);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double tgtPower;
        double Vertical;
        double Vertical_2;
        double horizontal_2;
        double horizontal;
        double Vertical_3;
        double Vertical_4;

        //boolean gp2AReleased = true;
        // boolean clawOpen = false;
        // boolean currentAbtn;
        //boolean currentYbtn;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        tgtPower = 0.6;
        Vertical = -gamepad1.right_stick_y * tgtPower;
        Vertical_2 = -gamepad1.left_stick_y * tgtPower;
        horizontal_2 = gamepad1.left_stick_x * tgtPower;
        horizontal = gamepad1.right_stick_x * tgtPower;
        Vertical_3 = gamepad2.left_stick_y * 1;
        Vertical_4 = gamepad2.left_stick_y * 1;

        robot.FRMotor.setPower(Vertical - horizontal);
        robot.BRMotor.setPower(Vertical + horizontal);
        robot.FLMotor.setPower(Vertical_2 + horizontal_2);
        robot.BLMotor.setPower(Vertical_2 - horizontal_2);
        robot.LSMotor.setPower(Vertical_3);
        robot.RSMotor.setPower(Vertical_4);

        // currentAbtn = gamepad2.a;
        // currentYbtn = gamepad2.y;
        if (gamepad1.y) {
            robot.DroneServo.setPosition(1);

            // robot.TiltServo.setPosition(0.50);
            // }// else if (gamepad1.x) {
            // robot.TiltServo.setPosition(0.51);
        } else if (gamepad1.left_bumper) { //open
            robot.CRServo.setPosition(0.43);
            robot.LCServo.setPosition(0.53);
        } else if (gamepad1.right_bumper) { //close
            robot.LCServo.setPosition(0.45);
            robot.CRServo.setPosition(0.5);
        }

        if (gamepad2.x) {
            robot.TRServo.setPosition(0.57); //tilt
        }
        if (gamepad2.a) {
            robot.TRServo.setPosition(0.545); //pickup
        } else if (gamepad2.y) {
            robot.TRServo.setPosition(0.45); //score
        }

      //  }
    /*    if (currentYbtn) {
            claw.move(Consts.Claw.CLOSECLAW);
        }
        else if (currentAbtn) {
            claw.move(Consts.Claw.OPENCLAW);
        } */

        // Toggle claw
      /*  currentAbtn = gamepad2.a;
        if (!currentAbtn) {
            gp2AReleased = true;
        }

        if (currentAbtn && gp2AReleased) {
            gp2AReleased = false;
            if (clawOpen) {
                Claw.move(Consts.Claw.CLOSECLAW);
                clawOpen = false;
            } else {
                Claw.move(Consts.Claw.OPENCLAW);
                clawOpen = true; */
        // Moving lift
      /*  if (gamepad2.dpad_up) {
            lift.move(Consts.Lift.HIGH);
        } else if (gamepad2.dpad_right) {
            lift.move(Consts.Lift.LOW);
        } else if (gamepad2.dpad_left) {
            lift.move(Consts.Lift.MEDIUM);
        } else if (gamepad2.dpad_down) {
            lift.move(Consts.Lift.ZERO);
        } */


        //telemetry.addData("rTrigger ", gamepad1.right_trigger);
        // telemetry.addData("lTrigger ", gamepad1.left_trigger);
        //telemetry.addData("clawOpen", clawOpen);
        // telemetry.addData("claw position ", claw.getPosition());
        //telemetry.addData("claw tilt ", Tilt.getPosition());
        /*telemetry.addData("right stick x pos", gamepad1.right_stick_x);
        telemetry.addData("right stick y pos", gamepad1.right_stick_y);

       // telemetry.addData("Lift Position", lift.getPosition());
        telemetry.addData("Dpad Up", gamepad2.dpad_up);
        telemetry.addData("Dpad right", gamepad2.dpad_right);
        telemetry.addData("Dpad down", gamepad2.dpad_down);
        telemetry.addData("Dpad left", gamepad2.dpad_left);
        telemetry.addData("gamepad 2 x button", gamepad2.x);
        telemetry.update(); */







        /*
         * Code to run ONCE after the driver hits STOP
         */
     //   @Override
      //  public void stop() {

      //  }


    }

}



