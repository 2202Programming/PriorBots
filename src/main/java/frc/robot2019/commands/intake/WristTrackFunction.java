package frc.robot2019.commands.intake; 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

public class WristTrackFunction extends Command{
    DoubleSupplier angleFunct;
    private DoubleSupplier offset;

    public WristTrackFunction(DoubleSupplier angleFunct){
        this(angleFunct, () -> 0.0);
    }

    public WristTrackFunction(DoubleSupplier angleFunct, DoubleSupplier offset){
        addRequirements(Robot.intake);
        this.angleFunct = angleFunct;
        this.offset = offset;
    }

    @Override
    public void execute() {
        //intake angle is relative to arm
        double angle = angleFunct.getAsDouble() + offset.getAsDouble();
        Robot.intake.setAngle(angle) ;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}