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
        //Drive Stick
        public static final int ARM_TOGGLE = 1;
        public static final int COLLECTOR_REVERSE = 2;
        
        //Op Stick
        public static final int HANGER_PNEUMATICS_TOGGLE = 4;
        public static final int SLIDER_AXIS = 3;
        public static final int AIM_TOGGLE = 3;
        public static final int SHOOT_ALLIANCE_BALL = 1;
        public static final int RUN_SHOOTER_TOGGLE = 2;
        public static final int SHOOT_WITH_SLIDER = 11;
        public static final int TURRET_LEFT = 9;
        public static final int SHOOTER_SPEED_DOWN = 12;
        public static final int SHOOTER_SPEED_UP = 10;
        public static final int TURRET_TO_ZERO = 7;
        public static final int ELEVATOR_MAX_UP = 0;
        public static final int ELEVATOR_MAX_DOWN = 180;
        public static final int REVERSE_LIFTER = 5;
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

        public static final int TURRET_MOTOR = 11;

        public static final int LF_MAIN = 10;

        public static final int CL_ARM = 9;
        public static final int CL_GATE_LEFT = 13;
        public static final int CL_GATE_RIGHT = 14;

        public static final int HA_LEFT = 15;
        public static final int HA_RIGHT = 16;

        public static final int PNEUMATICS_HUB = 21;

    }

    public final class PneumaticsIDs {
        //COLLECTOR_A is A on solenoid(currently the extending one)
        //COLLECTOR_B is B on solenoid(currently the retracting one)
        public static final int COLLECTOR_A = 0;
        public static final int COLLECTOR_B = 1;

        //HANGER_A is A on solenoid(currently the extending one)
        //HANGER_B is B on solenoid(currently the retracting one)
        public static final int HANGER_A = 2;
        public static final int HANGER_B = 3;

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

    public final class Turret {
        public static final int GEAR_RATIO = 10*4;
        public static final double DEFAULT_SPEED = 0.3;
        public static final double ZERO = 0;
        public static final double CLAMP = 0.3;
        
        public final class PID {
            public static final double kP = 0.0086;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double FF = 0.1;
            public static final double TOLERANCE = 1.0;
            public static final int TOLERANCE_BUFFER = 5;
        }
    }

    public final class Shooter {
        // Percent of max speed
        public static final double DEFAULT_SPEED = 0.58;
        public static final double ADJUST_INCREMENT = 0.05;
    }

    public final class Lifter {
        public static final double DEFAULT_SPEED = 0.5;
    }

    public final class Collector {
        public static final double ARM_SPEED = 0.4;
        public static final double GATE_SPEED = 0.5;
    }

    public final class Limelight {
        //Units = CM
        
        public static final double LIMELIGHT_HEIGHT = 40;
        public static final double TARGET_HEIGHT = 264;
        public static final double LIMELIGHT_ANGLE = 29.962;

        public static final double SEARCH_SPEED = 0.3;
    }

    public final class Etcetera {}

    public static final boolean WONKY = false;

}
