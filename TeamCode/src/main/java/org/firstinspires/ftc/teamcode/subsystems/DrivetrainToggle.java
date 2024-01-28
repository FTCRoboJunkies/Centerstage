
package org.firstinspires.ftc.teamcode.subsystems;

        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import java.util.Arrays;
        import java.util.List;

public class DrivetrainToggle {
    public static DcMotor BLMotor;
    public static DcMotor BRMotor;
    public static DcMotor FLMotor;
    public static DcMotor FRMotor;
    private static HardwareMap hwMap;


    private static List<DcMotor> motors;
    private static boolean isSlowMode = false;

    public static void init(HardwareMap ahwMap) {
        // Your existing initialization code remains unchanged
        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/

        // Control HUb
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");

        /**
         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
         * **/
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**
         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         * **/

        //THIS IS THE CORRECT ORIENTATION
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        /**
         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
         * **/

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //...ZeroPowerBehavior.FLOAT when you don't want it to stop
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        /**
         *The 4 mecanum wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button
         * **/
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);


        // Add all motors to the list
        motors = Arrays.asList(FLMotor, FRMotor, BLMotor, BRMotor);
    }

    public static void toggleSpeed() {
        // Toggle between slow and fast mode
        isSlowMode = !isSlowMode;

        // Set power based on the mode
        double powerMultiplier = isSlowMode ? 0.5 : 1.0;

        // Set power for all motors
        for (DcMotor motor : motors) {
            motor.setPower(motor.getPower() * powerMultiplier);
        }
    }

    public static void power(double output) {
        FRMotor.setPower(-output);
        FLMotor.setPower(-output);
        BRMotor.setPower(output);
        BLMotor.setPower(output);
    }

    // Add any additional methods or modifications as needed
}
