package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;



@Config
public class KookyRobotHardware {

    //drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    // extension






    public CRServoImplEx leftHang;
    public CRServoImplEx rightHang;

    private HardwareMap hardwareMap;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static KookyRobotHardware instance = null;
    private boolean enabled;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;


   /* private ArrayList<WSubsystem> subsystems;



    public MecanumDrivetrain drivetrain;
    public DroneSubsystem drone;
    public HangSubsystem hang;


    public ThreeWheelLocalizer localizer;


    public HashMap<Sensors.SensorType, Object> values;

   */ public static KookyRobotHardware getInstance() {
        if (instance == null) {
            instance = new KookyRobotHardware();
        }
        instance.enabled = true;
        return instance;
    }




    public void startCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

    public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }

    public void kill() {
        instance = null;
    }
}
