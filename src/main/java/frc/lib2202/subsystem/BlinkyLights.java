// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem;

import java.util.ArrayList;
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

    //removed static to support multiple uses on different light groups
    private BlinkyLightUser currentUser = null;

    // Some common colors
    static public Color8Bit BLACK = new Color8Bit(0, 0, 0);
    static public Color8Bit WHITE = new Color8Bit(255, 255, 255);
    static public Color8Bit RED = new Color8Bit(255, 0, 0);
    static public Color8Bit GREEN = new Color8Bit(0, 255, 0);
    static public Color8Bit BLUE = new Color8Bit(0, 0, 255);
    static public Color8Bit ORANGE = new Color8Bit(255, 145, 0);
    static public Color8Bit YELLOW = new Color8Bit(255, 255, 0);

    // State vars, handle N candle sets
    ArrayList<CANdle> m_candles;
    ArrayList<Color8Bit> colors;
    Color8Bit currentColor;

    final BlinkyLightUser defaultUser;

    public BlinkyLights(int... can_ids){
        m_candles = new ArrayList<CANdle>();
        colors = new ArrayList<Color8Bit>();
        defaultUser = new BlinkyLightUser(this) {
            @Override
            public Color8Bit colorProvider() {
                return ORANGE;
            }
        };

        for(int id : can_ids){
            var candle = new CANdle(id);
            config(candle);
            m_candles.add(candle);
            //default color
            colors.add(WHITE);
        }
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
        for(int i = 0; i < m_candles.size(); i++) {
            m_candles.get(i).setLEDs(color.red, color.green, color.blue);
            colors.set(i, color);
        }  
        currentColor = color;
    }
    void setIndividualColor(int CANdleNum, Color8Bit color) {
        if (CANdleNum < m_candles.size()) {
            m_candles.get(CANdleNum).setLEDs(color.red, color.green, color.blue);
            colors.set(CANdleNum, color);
        }
    }

    void setBlinking(boolean blink) {
        if (blink)
            setBlinking(currentColor);
        else
            stopBlinking();
    }

    void setBlinking(Color8Bit color) {
        Animation animation = new StrobeAnimation(color.red, color.green, color.blue, 0, 0.5, 8);
        for(CANdle c : m_candles) {
        c.animate(animation, 0);
        }
    }

    void stopBlinking() {
        for(CANdle c : m_candles) {
        c.clearAnimation(0);
        }
    }

    /*
     * Brightness on a scale from 0-1, with 1 being max brightness
     */
    void setBrightness(double brightness) {
        for(CANdle c : m_candles) {
            c.configBrightnessScalar(brightness);
        }        
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
        /**
         * Used in commands, override as you wish
         * Order: Color, CANdleNum
         * @return an object array that contains the Color to set to (first), and secondly, the CANdle number to set to
         */
        public ArrayList<Object> individualColorProvider() {
            ArrayList<Object> arr = new ArrayList<>();
            arr.add(WHITE);
            arr.add(0);
            return arr;
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
            Color8Bit newIndividualColor = (Color8Bit) myUser.individualColorProvider().get(0);
            // avoid CAN bus traffic if color isn't changing
            for(int i = 0; i < m_candles.size(); i++) {
                if (colors.get(i) != newIndividualColor) {
                    setIndividualColor((int) myUser.individualColorProvider().get(1), newIndividualColor);
                }
            }
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
