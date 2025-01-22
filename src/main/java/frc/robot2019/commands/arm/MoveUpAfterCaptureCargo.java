package frc.robot2019.commands.arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;

public class MoveUpAfterCaptureCargo extends Command {
    /*
    Constants should probably be moved to the subsystem?
    Many commands will be involved in moving the arm in an x-y coordinate system
    They will all require these constants to some extent
    */

    //Length of the arm from pivot point without extension in inches
    private final double armInitialLength = 30.0;
    
    private final double kTolerance = 1.0;

    //Projection of the arm on the ground to be maintained
    private double curProjection;

    private double curCalcHeight;
    private double endHeight;

    final ArmSubsystem arm;

    public MoveUpAfterCaptureCargo(){
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(arm);
        curProjection = (armInitialLength +arm.getExtension()) * Math.cos(arm.getRealAngle());
        curCalcHeight = Math.sqrt((arm.getExtension() + armInitialLength) * (arm.getExtension() + armInitialLength) - curProjection * curProjection);
        endHeight = curCalcHeight - 5.0; //Move up 5 inches (increase bc decreasing distance from x-axis)
    }

    public void execute() {
        //Move to the endHeight while maintaining curProjection
       arm.setAngle(90 + Math.toDegrees(Math.atan(endHeight / curProjection)));
        //Add 90 bc calc goes below x axis

       arm.setExtension(
            Math.sqrt(endHeight * endHeight + curProjection * curProjection) - armInitialLength);
    }

    public boolean isFinished() {
        return Math.abs(curCalcHeight - endHeight) < kTolerance;
    }
}