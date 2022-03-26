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
        public static final int HUNT_FOR_BALLS = 1;//ARM_TOGGLE = 1;
        public static final int COLLECTOR_REVERSE = 2;
        
        //Op Stick
        public static final int HANGER_PNEUMATICS_REVERSE = 4;
        public static final int HANGER_PNEUMATICS_FORWARD = 6;
        public static final int SLIDER_AXIS = 3;
        public static final int AIM_TOGGLE = 3;
        public static final int SHOOT_ALLIANCE_BALL = 1;
        public static final int RUN_SHOOTER_TOGGLE = 2;
        public static final int SHOOT_WITH_SLIDER = 11;
        public static final int TURRET_LEFT = 9;
        public static final int SHOOTER_SPEED_DOWN = 12;
        public static final int SHOOTER_SPEED_UP = 10;
        public static final int TURRET_TO_ZERO = 7;
        public static final int ELEVATOR_UP = 180;
        //public static final int ELEVATOR_CANCEL = 90;
        public static final int ELEVATOR_DOWN = 0;
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
        public static final int LOADER_MOTOR = 12;

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
        public static final double DEADBAND = 0.05;
        public static final double JOYSTICK_SPEED_FACTOR = 1;
        public static final double JOYSTICK_TURN_FACTOR = 0.6;//Change this to like 1
    }

    public final class Auto {
        public static final double DEFAULT_AUTO_TIME = 3.0;

        public static final double TWO_BALL_AUTO_DRIVE_DISTANCE = 4;//feet
        public static final double TWO_BALL_AUTO_DRIVE_SPEED = 0.4;
        public static final double TWO_BALL_AUTO_SHOOTER_SPEED = 2864;

        public static final double THREE_BALL_AUTO_FIRST_DISTANCE = 21; //feet
        public static final double THREE_BALL_AUTO_SECOND_DISTANCE = -17; //feet
        public static final double THREE_BALL_AUTO_FIRST_TURN = 105; //degrees
        public static final double THREE_BALL_AUTO_SECOND_TURN = 155; //degrees


        public static final double LIFTLOAD_AUTO_TIMEOUT = 1.5;

    }

    public final class Drivetrain {
        public static final double DRIVE_DEADBAND = 0.02;
        public static final double TURN_DEADBAND = 0.02;
        public static final double DRIVE_LIMITER = 50; //2
        public static final double TURN_LIMITER = 1.8; //1.5

        public static final double P_TURN = 0.005;
	    public static final double I_TURN = 0.001;
	    public static final double D_TURN = 0;
	    public static final double TOLERANCE_TURN = 2;			// degrees
	    public static final double TOLERANCE_TURN_RATE = 4;   	// degrees per second
	    public static final double FEED_TURN = 0.05;
    }

    public final class Turret {
        public static final int GEAR_RATIO = 10*4;
        public static final double DEFAULT_SPEED = 0.3;
        public static final double ZERO = 0;
        public static final double CLAMP = 0.8;//0.3;
        public static final double TURRET_LIMITER = 1;
        public static final double RAMP_RATE = 3;

        public final class PID {
            public static final double kP = 0.0055;//.0086;
            public static final double kI = 0.002;//0.0;
            public static final double kD = 0.0;
            public static final double FF = 0.1;
            public static final double TOLERANCE = 1.0;
            public static final int TOLERANCE_BUFFER = 5;
        }
    }

    public final class Shooter {
        // Percent of max speed
        public static final double SHOOTER_DEFAULT_SPEED = 3000;//0.58;
        public static final double ADJUST_INCREMENT = 0.05;
        public static final double MAX_SHOOTER_POWER = 0.75;
        public static final double SHOOTING_SPEED_THRESHOLD = 0.8;
    }

    public final class Lifter {
        public static final double LIFTER_DEFAULT_SPEED = 0.5;
        public static final double COLOR_CONFIDENCE = 0.93;
    }

    public final class Loader {
        public static final double LOADER_IDLE_SPEED = -0.1;
    }


    public final class Collector {
        public static final double COLLECTOR_DEFAULT_SPEED = 0.6;
    }

    public final class Gate {
        public static final double GATE_DEFAULT_SPEED = 0.3;
    }

    public final class Hanger {
        public static final double HANGER_DEFAULT_SPEED = 0.5;
        public static final double MAX_HEIGHT = 98;  // number of rotaions to the top
        public static final double HANGER_POSITION_TOLERANCE = 1;
        public static final double BAR_POSITION = 29;  // just below the fixed hooks
        public static final double MAX_PITCHRATE = 60; //OK to climb if below this

        
        public static final double HANGER_kP = 0.06;
        public static final double HANGER_kI = 0.00005;
        public static final double HANGER_kD = 0.02;

        public static final double HANGER_PULL_kP = 0.03;
        public static final double HANGER_PULL_kI = 0.005;
        public static final double HANGER_PULL_kD = 0.0;

        public static final double STEP_1 = 60;
        public static final double STEP_2 = 90;
        public static final double STEP_3 = 95;

        




    }


    public final class Limelight {
        //Units = CM
        
        public static final double LIMELIGHT_HEIGHT = 102;
        public static final double TARGET_HEIGHT = 259;
        public static final double LIMELIGHT_ANGLE = 29.962;

        public static final double SEARCH_SPEED = .1;//.3;
    }

    public final class Etcetera {}

    public static final boolean WONKY = false;

}
