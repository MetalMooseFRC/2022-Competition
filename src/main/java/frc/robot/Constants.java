// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Buttons {
        public static final int ARM_TOGGLE = 2;
        public static final int HANGER_PNEUMATICS_TOGGLE = 4;
    }

    public final class Limelight {
        //Units = CM
        
        public static final double LIMELIGHT_HEIGHT = 40;
        public static final double TARGET_HEIGHT = 264;
        public static final double LIMELIGHT_ANGLE = 29.962;

        public final class PID {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double VISION_ERROR = 0.0;
            public static final double TURRET_SETPOINT = 0.0;
        }
    }
    public final class CANIDs {
        public static final int DT_LEFT_BACK = 5;
        public static final int DT_LEFT_MIDDLE = 3;
        public static final int DT_LEFT_FRONT = 1;
        
        public static final int DT_RIGHT_FRONT = 2;
        public static final int DT_RIGHT_MIDDLE = 4;
        public static final int DT_RIGHT_BACK = 6; 
        
        public static final int SH_LEFT = 7;
        public static final int SH_RIGHT = 8;
        
        public static final int LF_MAIN = 10;

        public static final int CL_ARM = 9;
        public static final int CL_GATE_LEFT = 13;
        public static final int CL_GATE_RIGHT = 14;

        public static final int HA_LEFT = 15;
        public static final int HA_RIGHT = 16;

        public static final int PNEUMATICS_HUB = 21;

        public static final int TURRET_AIM = 11;

    }

    public final class PneumaticsIDs {
        //Collector_1 is A on solenoid(currently the extending one)
        //Collector_2 is B on solenoid(currently the retracting one)
        public static final int COLLECTOR_1 = 0;
        public static final int COLLECTOR_2 = 1;

        //Hanger_1 is A on solenoid(currently the extending one)
        //Hanger_2 is B on solenoid(currently the retracting one)
        public static final int HANGER_1 = 2;
        public static final int HANGER_2 = 3;

    }

    public final class DSPorts {
        public static final int DRIVER_STICK_PORT = 0;
        public static final int OPERATOR_STICK_PORT = 1;
    }

    public final class Preferences {
        public static final double DEADBAND = 0.15;
        public static final double JOYSTICK_SPEED_FACTOR = 1;
        public static final double JOYSTICK_TURN_FACTOR = 0.6;
    }

    public final class Shooter {
        // Percent of max speed
        public static final double DEFAULT_SPEED = 0.5;
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    public final class Lifter {
        public static final double DEFAULT_SPEED = 0.5;
        public static final double KP = 0.01;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    public final class Collector {
        public static final double ARM_SPEED = 0.4;
        public static final double GATE_SPEED = 0.4;
    }

    public final class Etcetera {}
    //    (/ oo \)
    //     \(__)/
    //     /|  |\
    public static final boolean WONKY = true;
    public static final int SCROMBLO = 4;
}
