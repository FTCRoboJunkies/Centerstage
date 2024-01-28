package org.firstinspires.ftc.teamcode.hardware;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HardwareTeleop {


    // Instantiate motors and servos
    public DcMotor BLMotor= null;
    public DcMotor BRMotor= null;
    public DcMotor FLMotor= null;
    public DcMotor FRMotor= null;

    public DcMotor LSMotor= null;
    public DcMotor RSMotor= null;



   public Servo LCServo = null;
   public Servo TRServo= null;
   public Servo DroneServo = null;

   public Servo CRServo = null;




    //Creating the Hardware Map
    HardwareMap hwMap =null;


    public HardwareTeleop(){

    }

    public void init(HardwareMap ahwMap)    {
        hwMap = ahwMap;



        //Define motors and servos
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        RSMotor = hwMap.get(DcMotor.class,"RSliderMotor");
        LSMotor = hwMap.get(DcMotor.class,"LSliderMotor");

        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RSMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LSMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo = hwMap.get(Servo.class, "CRServo");
        LCServo = hwMap.get(Servo.class, "CLServo");
        TRServo = hwMap.get(Servo.class, "TRServo");
        DroneServo= hwMap.get(Servo.class, "DroneServo");







        //Define Sensors
        //gyroSensor = hwMap.get(GyroSensor.class, "gyroSensor");

        //Set motor power
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
        LSMotor.setPower(0);
        LSMotor.setTargetPosition(0);
        RSMotor.setPower(0);
        RSMotor.setTargetPosition(0);

        //Set motor mode
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RSMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //set motor zeroPowerBehavior
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //...ZeroPowerBehavior.FLOAT when you don't want it to stop
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RSMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LSMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        //set servo power
        //IntakeServo.setPower(0);



        //set servo position


        CRServo.setPosition(0.5);
        LCServo.setPosition(0.45);
        TRServo.setPosition(0.43); //0.4 init/floor
        DroneServo.setPosition(0.05);






    }






}







