package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config("Constants")
public class Consts {
    /*
Lift Constants
*/



    public enum Lift {
        ZERO,
        LOW,
        MEDIUM,
        HIGH,
        AUTO_GROUND,
        AUTO_LOW,
        AUTO_MEDIUM,
        AUTO_HIGH,
        TRANSITION
    }


    /*
    Claw Constants
    */
    public enum Claw {
        LOPENCLAW,
        ROPENCLAW,
        LCLOSECLAW,
        RCLOSECLAW,


    }

    /*
    Claw Init Variables
     */
    //0 moves toward opens, 1 moves towards close
    public static double LCLAW_CLOSE_LIMIT = 0.5; // for slightly more precision
    public static double LCLAW_OPEN_LIMIT = 0.53;
    public static double RCLAW_CLOSE_LIMIT = 0.5; // for slightly more precision
    public static double RCLAW_OPEN_LIMIT = 0.43;


    /*
    Tilt Constants
     */
    public enum Tilt {
        SCORETILT,
        PICKUPTILT,
        ANGLETILT
    }
    /*
    Tilt Variables
     */
    //0 move left, 1 move right
    public static double TILT_FRONT_LIMIT = 0.4; //init point 0.4 or 0.45
    public static double TILT_ANGLE_DOWN = 0.43;//down angle to be able to fit
    public static double TILT_BACK_LIMIT = 0.38;//score/drop position

    /*
    Intake Constants
     */
    public enum Intake {
        ON, OFF

    }
    /*
    Intake Variables
     */
    public static double INTAKE_ON = 1;
    public static double INTAKE_OFF = 0;
    /*
    Camera Constraints
     */
    //public enum AllianceType {
     //   BLUE, RED
    //}

    /*
    Camera Variables
     */

    /*
    Launcher Constraints
     */

    public enum Drone{
        Reload,
        Launch,
    }
    /*
    Launcher Variables
     */
    public static double Launcher_Launch = 1;
    public static double Launcher_Reload = 0.05;

    public  static double  ALLIANCE_TYPE;
}


