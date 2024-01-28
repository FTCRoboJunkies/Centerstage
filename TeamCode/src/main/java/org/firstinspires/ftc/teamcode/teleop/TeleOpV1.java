package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.Consts.Claw.LOPENCLAW;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.Claw.ROPENCLAW;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

//import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lclaw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LeftSlider;
import org.firstinspires.ftc.teamcode.subsystems.Rclaw;
import org.firstinspires.ftc.teamcode.subsystems.RightSlider;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;


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
@TeleOp(name = "🟢Blur")
@Config()
//@Disabled
public class TeleOpV1 extends LinearOpMode {


    //  Trajectory currentATrajectory;
    //  Trajectory currentBTrajectory;
    //  Pose2d poseEstimate;
    //  Pose2d startingPos = new Pose2d(12, 62, Math.toRadians(-90));
    // boolean followingTrajectory = false;
    Drivetrain drivetrain = new Drivetrain();
    public static DcMotor LSliderMotor, RSliderMotor;
    List<DcMotor> Slider;
    RightSlider rightSlider = new RightSlider();
    LeftSlider leftSlider = new LeftSlider();





    Lclaw lclaw = new Lclaw();
    Rclaw rclaw = new Rclaw();





    Drone drone = new Drone();

    Tilt tilt = new Tilt();


    @Override
    public void runOpMode() throws InterruptedException {
       // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      //  drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    /*    LSliderMotor = hardwareMap.get(DcMotorEx.class, "LSliderMotor");
        LSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       LSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RSliderMotor = hardwareMap.get(DcMotorEx.class, "RSliderMotor");
        RSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Slider =  Arrays.asList(LSliderMotor, RSliderMotor);
 */

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Drivetrain.init(hardwareMap);

       // intake.init(hardwareMap);

       lclaw.init(hardwareMap);
       rclaw.init(hardwareMap);
       rightSlider.init(hardwareMap);
       leftSlider.init(hardwareMap);

        //lift.init(hardwareMap);

        Drone.init(hardwareMap);

        tilt.init(hardwareMap);


        //drive.setPoseEstimate(startingPos);

        waitForStart();



        while (!isStopRequested()/* && !lift.requestStop */) {

            // drive.update();
            double Vertical;
            double Vertical_2;
            double horizontal_2;
            double horizontal;
            double Vertical_3;
            double Vertical_4;


            // Translation
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


           /* double tgtPower = 0.6;
            Vertical = -gamepad1.right_stick_y * tgtPower;
            Vertical_2 = -gamepad1.left_stick_y * tgtPower;
            horizontal_2 = gamepad1.left_stick_x * tgtPower;
            horizontal = gamepad1.right_stick_x * tgtPower;
            Vertical_3 = gamepad2.left_stick_y * 0.8;
            Vertical_4 = gamepad2.left_stick_y * 0.8;


            FRMotor.setPower(Vertical - horizontal);
            BRMotor.setPower(Vertical + horizontal);
            FLMotor.setPower(Vertical_2 + horizontal_2);
            BLMotor.setPower(Vertical_2 - horizontal_2);
            RSliderMotor.setPower(Vertical_3);
            LSliderMotor.setPower(Vertical_4); */


            if (gamepad1.y) {
                Drone.move(Consts.Drone.Launch);
            }
          /*  if (gamepad1.y) {
                Drone.move(Consts.Drone.Reload);
            } */


            // Toggle claw
            // Toggle claw with bumpers
         /*   if (gamepad1.left_bumper && !gamepad1_last.left_bumper) {
                // Toggle claw state when the left bumper is pressed
                clawOpen = !clawOpen;
                Claw.move(clawOpen ? Consts.Claw.OPENCLAW : Consts.Claw.CLOSECLAW);
            }

            gamepad1_last.copy(gamepad1); // Update the previous state of gamepad1 */




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




            // Toggle belt
            // currentBbtn = gamepad2.b;
            //  if (!currentBbtn) {
            //       gp2BReleased = true;
            // }

            // if (currentBbtn && gp2BReleased) {
            // gp2BReleased = false;

            //  }


            // Turntable rotation


            // currentRBumper = gamepad2.right_bumper;

            //    if (!currentRBumper) {
            //      gp2RBumperReleased = true;
            //  }

            // if (currentRBumper && gp2RBumperReleased) {
            //    gp2RBumperReleased = false;

//                else if (tableRotation < 270){
//                    tableRotation = 270;
//                }
            //   }

            // currentLBumper = gamepad2.left_bumper;
            //  if (!currentLBumper) {
            //      gp2LBumperReleased = true;
            //  }

            //  if (currentLBumper && gp2LBumperReleased) {
            //      gp2LBumperReleased = false;


//                else if (tableRotation > -270){
//                    tableRotation = -270;


            // Moving lift
            if (gamepad2.dpad_up) {

                LeftSlider.move(Consts.Lift.HIGH);
                RightSlider.move(Consts.Lift.HIGH);
               // Tilt.move(Consts.Tilt.ANGLETILT);
              //  sleep(3000);
               // Tilt.move(Consts.Tilt.SCORETILT);
            } else if (gamepad2.dpad_right) {

                LeftSlider.move(Consts.Lift.LOW);
                RightSlider.move(Consts.Lift.LOW);
               // Tilt.move(Consts.Tilt.ANGLETILT);
               // sleep(3000);
                //Tilt.move(Consts.Tilt.SCORETILT);

            } else if (gamepad2.dpad_left) {

                LeftSlider.move(Consts.Lift.MEDIUM);
                RightSlider.move(Consts.Lift.MEDIUM);
                //Tilt.move(Consts.Tilt.ANGLETILT);
               // sleep(3000);
               // Tilt.move(Consts.Tilt.SCORETILT);

            } else if (gamepad2.dpad_down) {

                LeftSlider.move(Consts.Lift.ZERO);
                RightSlider.move(Consts.Lift.ZERO);
               // Tilt.move(Consts.Tilt.ANGLETILT);
                //sleep(2000);
               // Tilt.move(Consts.Tilt.PICKUPTILT);
            }


            // X: reset subsystems for intaking action
            // turn table turned
            // lift up
            // claw open
            // belt is down
            // belt up -> claw close -> turntable turn back -> lift down

          /*  if (gamepad2.x) {


                Tilt.move(Consts.Tilt.PICKUPTILT);
                sleep(1000);
                Lift.move(Consts.Lift.ZERO);
                Launcher.move(Consts.Launcher.Reload);
            } */


            //  if (gamepad1.a && !gp1APressed && !followingTrajectory) {
            // gp1APressed = true;
            //  followingTrajectory = true;
//                currentATrajectory = drive.trajectoryBuilder(poseEstimate).lineTo(new Vector2d(12, 22.5)).build();
            //  currentATrajectory = drive.trajectoryBuilder(poseEstimate).forward(33.5).build();
            //  drive.followTrajectoryAsync(currentATrajectory);
            //  }
            //  if (!gamepad1.a) {
            //     gp1APressed = false;
            //  }

            //  if (gamepad1.b && !gp1BPressed && !followingTrajectory) {
            // gp1BPressed = true;
            // followingTrajectory = true;
//                currentBTrajectory = drive.trajectoryBuilder(poseEstimate).lineTo(new Vector2d(12, 56)).build();
            //  currentBTrajectory = drive.trajectoryBuilder(poseEstimate).back(33.5).build();
            // drive.followTrajectoryAsync(currentATrajectory);
            //  }
            //   if (!gamepad1.b) {
            //       gp1BPressed = false;
            // }
            //    if (followingTrajectory) {
            //  drive.update();
            //  }
            // if (!drive.isBusy()){
            //  followingTrajectory = false;
            //  }

            //  if (!followingTrajectory){
            //     drive.setWeightedDrivePower(new Pose2d());


            // telemetry.addData("rTrigger ", gamepad1.right_trigger);
            // telemetry.addData("lTrigger ", gamepad1.left_trigger);
            telemetry.addData("clawOpen", LOPENCLAW);
            telemetry.addData("clawOpen", ROPENCLAW);
          //  telemetry.addData("claw position ", Claw.getPosition());
            // telemetry.addData("x", poseEstimate.getX());
            //  telemetry.addData("y", poseEstimate.getY());
            //  telemetry.addData("heading", poseEstimate.getHeading());
            //  telemetry.addData("right stick x pos", gamepad1.right_stick_x);
            // telemetry.addData("right stick y pos", gamepad1.right_stick_y);
            //  telemetry.addData("rotation", rotation);
            // telemetry.addData("Belt Position", belt.getPosition());
           // telemetry.addData("Lift Position", Lift.getPosition());
            telemetry.addData("Dpad Up", gamepad2.dpad_up);
            telemetry.addData("Dpad right", gamepad2.dpad_right);
            telemetry.addData("Dpad down", gamepad2.dpad_down);
            telemetry.addData("Dpad left", gamepad2.dpad_left);
            telemetry.addData("gamepad 2 x button", gamepad2.x);


            telemetry.update();


        }
    }
}



