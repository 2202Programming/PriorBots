package frc.robot2023.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot2023.subsystems.BlinkyLights.BlinkyLightUser;

public class ContinuousBlink extends Command implements BlinkyLightUser {

    private final boolean blink;
    private final Color8Bit color;

    public ContinuousBlink(Color8Bit color, Boolean blink) {
        this.blink = blink;
        this.color = color;
    }

    @Override
    public boolean requestBlink() {
        return blink;
    }

    @Override
    public Color8Bit colorProvider() {
        return color;
    }

    @Override
    public void initialize() {
        enableLights();
    }
}
