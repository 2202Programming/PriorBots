package frc.robot2019.commands.intake; 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;


public class WristTrackFunction extends Command{
    DoubleSupplier angleFunct;
    private DoubleSupplier offset;

    final IntakeSubsystem intake;
    public WristTrackFunction(DoubleSupplier angleFunct){
        this(angleFunct, () -> 0.0);
    }

    public WristTrackFunction(DoubleSupplier angleFunct, DoubleSupplier offset){
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake);

        this.angleFunct = angleFunct;
        this.offset = offset;
    }

    @Override
    public void execute() {
        //intake angle is relative to arm
        double angle = angleFunct.getAsDouble() + offset.getAsDouble();
        intake.setAngle(angle) ;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}