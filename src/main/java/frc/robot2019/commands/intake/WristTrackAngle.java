package frc.robot2019.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.IntakeSubsystem;

public class WristTrackAngle extends Command {
    private DoubleSupplier angleSupplier;
    final IntakeSubsystem intake;
    final ArmSubsystem arm;

    /**
     * Makes the wrist track a specific angle from vertical
     */
    public WristTrackAngle(double trackedAngle) {
        this(() -> trackedAngle);
    }

    public WristTrackAngle(DoubleSupplier trackedAngle) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(intake);
        angleSupplier = trackedAngle;
    }

    @Override
    public void execute() {
        // intake angle is relative to arm
        double offset = arm.getRealAngle() - angleSupplier.getAsDouble();
        intake.setAngle(offset);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}