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

import org.firstinspires.ftc.teamcode.OpenCV.Location;
import org.firstinspires.ftc.teamcode.OpenCV.PropPipelinev3;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.KookyRobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.vision.VisionPortal;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;

@Config
@Autonomous(name="🔴TestAuto2", group="Robot")
public class TestAuto2 extends LinearOpMode {

    private final KookyRobotHardware robot = KookyRobotHardware.getInstance();
    private FtcDashboard dashboard;

    private PropPipelinev3 propPipelinev3;
    private VisionPortal portal;
    private Location randomization;

    private double loopTime = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private double endTime = 0;


    Tilt tilt = new Tilt();
    Pose2d startingPos = new Pose2d(11, -70, 0);

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startingPos);


        tilt.init(hardwareMap);

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        propPipelinev3 = new PropPipelinev3();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipelinev3)
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
            telemetry.addData("position", propPipelinev3.getLocation());
            telemetry.update();
        }

        randomization = propPipelinev3.getLocation();
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
                Trajectory ParkR = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(ParkR);
                break;
        }
    }
}
