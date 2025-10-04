// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.timbot;

/**
 * The Constants class provides a convenient place for teams to hold timbot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    

    public static final class CAN { 
        public static final int ROBORIO = 0;
        public static final int PDP = 1;  // for rev
        public static final int PCM1 = 2; // for rev

        public static final int FLYWHEEL_TALON1 = 10;
        public static final int FLYWHEEL_TALON2 = 11;
        
        // TODO - verify this is a sparkmax and chan
        public static final int ACTUATOR = 12;  // 18??

        public static final int PIGEON_IMU_CAN = 60;
    }

    public static final class PWM{        
    }

    public static final class AnalogIn {
        public static final int LifterFeedback = 0;
      }
    
    public final class DigitalIO { 

    }

    // pneumatics control
    public static final class PCM {
        public static final int TRIGGER_FORWARD = 0;
        public static final int TRIGGER_BACK = 1;
    }

}
