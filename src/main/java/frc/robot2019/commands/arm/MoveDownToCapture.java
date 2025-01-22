package frc.robot2019.commands.arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;

public class MoveDownToCapture extends Command {
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
    private double down_cmd;   //inches to move down for pickup
    final ArmSubsystem arm;
    
    public MoveDownToCapture(double down){
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(arm);
        down_cmd = down;        
    }

    public void initialize() {
        curProjection = (armInitialLength +arm.getExtension()) * Math.cos(arm.getRealAngle());
        curCalcHeight = Math.sqrt((arm.getExtension() + armInitialLength) * (arm.getExtension() + armInitialLength) - curProjection * curProjection);
        endHeight = curCalcHeight + down_cmd; //Move down 5 inches (increase bc increasing distance from x-axis)
        /*
        Alternative method of calculating height
        
        */
    }

    // A button will trigger this when the pilot expects to be over a hatch/cargo.
    // We need to move into capturing mode, move down. wait a bit
    public void execute() {
        double angle = 90 + Math.toDegrees(Math.atan(endHeight / curProjection));
        double extension = Math.sqrt(endHeight * endHeight + curProjection * curProjection) - armInitialLength;
        //Move to the endHeight while maintaining curProjection
       arm.setAngle(angle);
        //Add 90 bc calc goes below x axis

       arm.setExtension(extension);

    }

    public boolean isFinished() {
        return Math.abs(curCalcHeight - endHeight) < kTolerance;
    }
}