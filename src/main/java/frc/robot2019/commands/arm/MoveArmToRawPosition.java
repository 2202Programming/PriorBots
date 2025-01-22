package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.Constants;
import frc.robot2019.subsystems.ArmSubsystem;

//Commands the arm to follow an arc
public class MoveArmToRawPosition extends Command {
    private double endAngle;
    private double extension;
    private double curAngle;
    private double tolerance;
    private double step;
    private double degreesPerSecond;
    final ArmSubsystem arm;

    public MoveArmToRawPosition(double angle, double extension, double endTolerance, double degreesPerSecond) {
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(arm);
        this.endAngle = angle;
        this.extension = extension;
        this.degreesPerSecond = degreesPerSecond;
        tolerance = endTolerance;
    }

    @Override
    public void initialize() {
        curAngle = arm.getRealAngle();
        step = Math.copySign(degreesPerSecond * Constants.dT, endAngle - curAngle); // step is the # of
                                                                                            // degrees to change per
                                                                                            // cycle
    }

    @Override
    public void execute() {
        arm.setExtension(extension);
        if (Math.abs(arm.getExtension() - extension) <= 0.5 || arm.isExtensionOverrided()) {
            arm.setAngle(curAngle);

            if (step < 0) {
                curAngle = Math.max(curAngle + step, endAngle);
            } else {
                curAngle = Math.min(curAngle + step, endAngle);
            }
        }
    }

    public boolean isFinished() {
        return Math.abs(arm.getRealAngle() - endAngle) <= tolerance;
    }

}