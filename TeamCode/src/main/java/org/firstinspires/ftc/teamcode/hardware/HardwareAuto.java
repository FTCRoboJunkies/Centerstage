package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.ftccommon.configuration.RobotConfigMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareAuto {


    // Instatiate motors and servos
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public CRServo IntakeServo = null;
    public Servo ClawServo = null;
    public Servo TiltServo = null;


    //Creating the Hardware Map
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private RobotConfigMap modules;

    public HardwareAuto() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        //Define motors and servos
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");

        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IntakeServo = hwMap.get(CRServo.class, "IntakeServo");
        ClawServo = hwMap.get(Servo.class, "ClawServo");
        TiltServo = hwMap.get(Servo.class, "TiltServo");


        IntakeServo.setDirection(CRServo.Direction.FORWARD);

        //Define Sensors
        //gyroSensor = hwMap.get(GyroSensor.class, "gyroSensor");

        //Set motor power
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);

        //Set servo power
        IntakeServo.setPower(0);

        //Set motor mode
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set motor zeroPowerBehavior
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //...ZeroPowerBehavior.FLOAT when you don't want it to stop
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //set servo position
        ClawServo.setPosition(0.40);
        TiltServo.setPosition(0.77);


    }
}










