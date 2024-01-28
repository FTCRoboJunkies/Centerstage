package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private static DcMotor BLMotor;
    private static DcMotor BRMotor;
    private static DcMotor FLMotor;
    private static DcMotor FRMotor;
    private static HardwareMap hwMap;
    private static boolean isSlowMode = false;

    public static void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Initialize motors
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");

        // Set motor directions
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Stop motors
        stop();
    }

    public static void power(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        double speedMultiplier = isSlowMode ? 0.5 : 1.0;
        FRMotor.setPower(-frontRightPower * speedMultiplier);
        FLMotor.setPower(-frontLeftPower * speedMultiplier);
        BRMotor.setPower(backRightPower * speedMultiplier);
        BLMotor.setPower(backLeftPower * speedMultiplier);
    }

    public static void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public static boolean isSlowMode() {
        return isSlowMode;
    }

    public static void stop() {
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
    }
}
