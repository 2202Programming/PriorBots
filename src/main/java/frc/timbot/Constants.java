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

    public static final class Elevation {

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
         * or 3. Only the first two (0,1) are visible in web-based configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
         * we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /* Choose so that Talon does not report sensor out of phase */
        public static boolean kSensorPhase = true;

        /**
         * Choose based on what direction you want to be positive, this does not affect
         * motor invert.
         */
        public static boolean kMotorInvert = false;

        /**
         * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki,
         * kd, kf, izone, peak output);
         */
        public static final Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);
    }

    public static final class SHOOTER {
        public static final double MIN_SHOOTER_SPEED = 200; // One unit represents one position unit per 100ms
    }

    public static final class CAN {
        public static final int FLYWHEEL_TALON1 = 10;
        public static final int FLYWHEEL_TALON2 = 11;

        public static final int ACTUATOR_TALON = 12;

        public static final int PCM = 1;

    }

    // pneumatics control
    public static final class PCM {
        public static final int TRIGGER_FORWARD = 0;
        public static final int TRIGGER_BACK = 1;
    }

}
