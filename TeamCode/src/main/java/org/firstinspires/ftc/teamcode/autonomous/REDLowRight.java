/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.PropPipelinev2;
import org.firstinspires.ftc.teamcode.OpenCV.Location;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.KookyRobotHardware;

import org.firstinspires.ftc.teamcode.subsystems.LeftSlider;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.vision.VisionPortal;


/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@Autonomous(name="🔴REDLowRight", group="Robot")
//@Disabled
public class REDLowRight extends LinearOpMode {

    private final KookyRobotHardware robot = KookyRobotHardware.getInstance();
    private FtcDashboard dashboard;

    private PropPipelinev2 propPipelinev2;
    private VisionPortal portal;
    private Location randomization;

    /* Declare OpMode members. */


    private double loopTime = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private double endTime = 0;


   // Claw claw = new Claw();
    LeftSlider lift = new LeftSlider();
    Tilt tilt = new Tilt();
    Pose2d startingPos = new Pose2d(11, -70, 0);





    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startingPos);



      //  claw.init(hardwareMap);

        lift.init(hardwareMap);


        tilt.init(hardwareMap);


        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        //drive.setDrivePower(new Pose2d(11, -70, 0));



        propPipelinev2 = new PropPipelinev2();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Set the camera
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipelinev2) // Add image processing pipeline
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        while (robot.getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propPipelinev2.getLocation());
            telemetry.update();
        }

        randomization = propPipelinev2.getLocation();
        portal.close();


        switch (randomization) {
            case LEFT:
                Trajectory left = drive.trajectoryBuilder(new Pose2d())
                        .back(40)

                        .splineToConstantHeading(new Vector2d(51, -35), Math.toRadians(0))
                        .build();
                drive.followTrajectory(left);


                break;
            case CENTER:
                Trajectory center = drive.trajectoryBuilder(new Pose2d())
                        .back(20)


                        .splineToConstantHeading(new Vector2d(51, -35), Math.toRadians(0))
                        .build();
                drive.followTrajectory(center);

                break;
            case RIGHT:
                Trajectory right = drive.trajectoryBuilder(new Pose2d())
                        .back(10)

                        .splineToConstantHeading(new Vector2d(51, -35), Math.toRadians(0))
                        .build();
                drive.followTrajectory(right);


                break;
            case NONE:
                // default
                Trajectory ParkR = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(ParkR);




                break;

        }
    }
}




