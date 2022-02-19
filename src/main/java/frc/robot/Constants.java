// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

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

    }

    public final class CANIDs {
        public static final int DT_LEFT_BACK = 1;
        public static final int DT_LEFT_MIDDLE = 2;
        public static final int DT_LEFT_FRONT = 3;
        
        public static final int DT_RIGHT_FRONT = 4;
        public static final int DT_RIGHT_MIDDLE = 5;
        public static final int DT_RIGHT_BACK = 6; 
        
        public static final int SH_LEFT = 7;
        public static final int SH_RIGHT = 8;
    }

    public final class DSPorts {
        public static final int DRIVER_STICK_PORT = 0;
        public static final int OP_STICK_PORT = 1;
    }

    public final class Preferences {
        public static final double DEADBAND = 0.07;
        public static final double JOYSTICK_SPEED_FACTOR = 1;
        public static final double JOYSTICK_TURN_FACTOR = 0.6;
    }
    public final class DefaultValues{
    }
    public final class Shooter {
        public static final double DEFAULT_SPEED = 10;
        public static final double KP = 0.2;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }
    public final class Etcetera {
        public static final int NEO_MAX_RPM = 5676;
    }

    public static final boolean WONKY = true;

}
