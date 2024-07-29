// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem;

import java.util.Optional;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;

public class BlinkyLights extends SubsystemBase {
    // variables and constants
    static final int TO = 50; // [ms] timeout for comms

    // candle frame update rate, larger saves on CAN BUS
    static final int FrameStatusTime = 200; // [ms]

    private static BlinkyLightUser currentUser = null;

    // Some common colors
    static public Color8Bit BLACK = new Color8Bit(0, 0, 0);
    static public Color8Bit WHITE = new Color8Bit(255, 255, 255);
    static public Color8Bit RED = new Color8Bit(255, 0, 0);
    static public Color8Bit GREEN = new Color8Bit(0, 255, 0);
    static public Color8Bit BLUE = new Color8Bit(0, 0, 255);
    static public Color8Bit ORANGE = new Color8Bit(255, 145, 0);
    static public Color8Bit YELLOW = new Color8Bit(255, 255, 0);

    // State vars
    CANdle candle_l;
    CANdle candle_r;
    Color8Bit currentColor;

    final BlinkyLightUser defaultUser;

    // constructor
    public BlinkyLights(int id_left, int id_right) {
        defaultUser = new BlinkyLightUser(this) {
            @Override
            public Color8Bit colorProvider() {
                return ORANGE;
            }
        };
        //configure hardware
        candle_l = new CANdle(id_left);
        candle_r = new CANdle(id_right);
        config(candle_l);
        config(candle_r);

        //set to default user's requests
        setCurrentUser(defaultUser);
        setColor(currentUser.colorProvider());
        setBrightness(1.0);
    }

    // blinkylights config
    void config(CANdle cdl) {
        final var StatusFrame = CANdleStatusFrame.CANdleStatusFrame_Status_1_General;

        var cfg = new CANdleConfiguration();
        cdl.clearStickyFaults(TO);
        cfg.enableOptimizations = true;
        cdl.configAllSettings(cfg, TO);

        // lower CAN bus usage for CANdle
        var period = cdl.getStatusFramePeriod(StatusFrame);
        if (period < FrameStatusTime)
            cdl.setStatusFramePeriod(StatusFrame, FrameStatusTime, TO);

        cdl.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, FrameStatusTime);
        cdl.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, FrameStatusTime);
    }

    // methods
    public void setCurrentUser(BlinkyLightUser user) {
        currentUser = user;
    };

    public void release() {
        currentUser = defaultUser;
        setColor(ORANGE);
    }

    /* Static methods to intercept robot state changes */
    public void onRobotInit() {
        currentUser.onRobotInit();
    };

    public void onDisabledPeriodic() {
        // always do our alliance colors
        setAllianceColors();
    }

    public void onAutomousdInit() {
        currentUser.onAutomousInit();
    };

    public void onTeleopInit() {
        currentUser.onTeleopInit();
    };

    public void onTestInit() {
        currentUser.onTestInit();
    };

    public void onTestPeriodic() {
        currentUser.onTestInit();
    };

    void setColor(Color8Bit color) {
        candle_l.setLEDs(color.red, color.green, color.blue);
        candle_r.setLEDs(color.red, color.green, color.blue);
        currentColor = color;
    }

    void setBlinking(boolean blink) {
        if (blink)
            setBlinking(currentColor);
        else
            stopBlinking();
    }

    void setBlinking(Color8Bit color) {
        Animation animation = new StrobeAnimation(color.red, color.green, color.blue, 0, 0.5, 8);
        candle_l.animate(animation, 0);
        candle_r.animate(animation, 0);
    }

    void stopBlinking() {
        candle_l.clearAnimation(0);
        candle_r.clearAnimation(0);
    }

    /*
     * Brightness on a scale from 0-1, with 1 being max brightness
     */
    void setBrightness(double brightness) {
        candle_l.configBrightnessScalar(brightness);
        candle_r.configBrightnessScalar(brightness);
    }

    public void setAllianceColors() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // no alliance indicated
        if (alliance.isEmpty()) {
            setColor(BLACK);
            return;
        }

        // System.out.println("***** Robot Alliance: " +
        // DriverStation.getAlliance().name());
        switch (alliance.get()) {
            case Blue:
                setColor(BLUE);
                break;
            case Red:
                setColor(RED);
                break;
            default:
                setColor(new Color8Bit(0, 255, 0));
                break;
        }
    }

    static boolean differentColors(Color8Bit c1, Color8Bit c2) {
        return (c1.red != c2.red) || (c1.green != c2.green) || (c1.blue != c2.blue);
    }

    /**
     * BlinkyLightUser a command for controllering of the lights
     */
    // commands and sub-systems
    public static class BlinkyLightUser extends Command {

        final BlinkyLights lights;

        public BlinkyLightUser() {
            lights = RobotContainer.getObjectOrNull("LIGHTS");
        }

        BlinkyLightUser(BlinkyLights lights) {
            this.lights = lights;
        }

        public void onRobotInit() {
        };

        public void onAutomousInit() {
        };

        public void onTeleopInit() {
        };

        public void onTestInit() {
        };

        // used in commands, Override to your preferences
        public Color8Bit colorProvider() {
            return WHITE;
        };

        // used in commands, Override to your preferences
        public boolean requestBlink() {
            return false;
        }

        public void enableLights() {
            // user is a Command, setup a parallel cmd to get the color info
            if (lights != null) {
                var watchCmd = lights.new UserWatcherCommand(this);
                watchCmd.schedule();
            }

        }
    } // blinkylights user

    class UserWatcherCommand extends Command {
        Color8Bit currentColor;
        boolean blinkState;
        final BlinkyLightUser myUser; // a command that is using the lights
        final Command watcherCmd; // a watcher to get values to the CANdles periodically

        UserWatcherCommand(BlinkyLightUser user) {
            myUser = user;
            watcherCmd = this;
            currentColor = user.colorProvider();
        }

        /*
         * Reads providers and send to CANDles.
         * 
         * This can could be setup to happen less frequently
         */
        @Override
        public void execute() {
            Color8Bit newColor = myUser.colorProvider();

            // avoid CAN bus traffic if color isn't changing
            if (!currentColor.equals(newColor)) {
                currentColor = newColor;
                setColor(currentColor);
            }
            if (myUser.requestBlink() != blinkState) {
                blinkState = myUser.requestBlink();
                setBlinking(blinkState);
            }
        }

        // watcher should always be able to
        @Override
        public boolean runsWhenDisabled() {
            return myUser.runsWhenDisabled();
        }

        @Override
        public void end(boolean interrupted) {
            release();
        }

        @Override
        public boolean isFinished() {
            // run until my parent command is done
            return !myUser.isScheduled();
        }
    }
}
