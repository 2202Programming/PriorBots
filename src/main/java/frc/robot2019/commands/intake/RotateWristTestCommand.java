package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.OI;
import frc.robot2019.subsystems.IntakeSubsystem;

/**
 * This command is intended to be used for testing purposes.
 * This command turns the wrist from zero, to positive, then to zero, then to negative, then to zero.
 * 
 * @author Kevin Li
 * 
 * 2/13/2019  dpl    removed unneeded overrides functions, is finished false to keep running.
 *
 * 
 */
public class RotateWristTestCommand extends Command {
  private XboxController ctrl;
  private double[] positions = {-30, 0, 30, 0};
  private int currentIndex = 0;
  final IntakeSubsystem intake;
  final OI m_oi;
  
  public RotateWristTestCommand() {
     intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
     m_oi = RobotContainer.getObject("OI");
     ctrl = m_oi.getAssistantController();
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setAngle(positions[currentIndex]);
  }

  @Override
  public void execute() {
    if (ctrl.getAButtonReleased()) { // TODO: change to respective button
      intake.setAngle(positions[currentIndex++]);
      if (currentIndex > positions.length) currentIndex = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return false;   //keep doing this until stopped
  }
}