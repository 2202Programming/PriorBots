// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot2025.Constants.DigitalIO;

/** Add your docs here. */

public class SignalLight {

    public enum Color { 
        OFF(0),
        GREEN(1),
        BLUE(2),
        TURQUOISE(3),
        RED(4),
        YELLOW(5),
        MAGENTA(6),
        WHITE(7);

        final private int color;
        Color(int b) {
            this.color = b;
        }
       public int color() {return color;}
    }

    // DIO that drive the light, needs 3
    final DigitalOutput d0;
    final DigitalOutput d1;
    final DigitalOutput d2;

    public SignalLight() {
        this(DigitalIO.SignalLight1, DigitalIO.SignalLight2, DigitalIO.SignalLight3);
    }

    public SignalLight(int dio1, int dio2, int dio3) {
        d0 = new DigitalOutput(dio1);
        d1 = new DigitalOutput(dio2);
        d2 = new DigitalOutput(dio3);
    }

    public void setLight(Color c){ 
        setLight(c.color());
    }

    public void setLight(int b) {
        d0.set((b & 0x01) != 0);
        d1.set((b & 0x02) != 0);
        d2.set((b & 0x04) != 0);
    }

    public Command getColorCommand(Color c) {
        return new InstantCommand(() -> {
            setLight(c);
        } );
    }


    public Command getDemoCommand() {
        return this.new DemoCommand();
    }

    public class DemoCommand extends Command {
        int color;
        int frameCount;

        /** Creates a new SetLight. */
        public DemoCommand() { }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            frameCount = 0;
            color = 0;
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() { 
            if (frameCount++ % 50 == 0) {
                color++;
                setLight(color);
            }
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }
    }

}