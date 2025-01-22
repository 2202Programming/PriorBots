package frc.robot2019.commands.arm;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.util.MathUtil;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.IntakeSubsystem;

/** 
 * 
 *   Billy & X think this is working for forward facing arm.
 * 
 *   TODO:FIX this to take account for which side arm is on
 * 
 *   2/20/19   DPL   adjusted to take h and l commands from functions
 *                   cleaned up constraints
 */

public class MoveArmAtHeight extends Command {
    final ArmSubsystem  arm; 
    final IntakeSubsystem intake;

    // Length of the arm from pivot point without extension in inches
    @SuppressWarnings("unused")
    private final double armInitialLength;
    // moved to inside ctor so arm/intake are initialized, otherwise make static
    // = arm.EXTEND_MIN + arm.ARM_BASE_LENGTH + intake.WristDistToPivot + arm.L0;

    // Height of point of rotation for the arm in inches
    private final double pivotHeight; // = arm.ARM_PIVOT_HEIGHT;

    // Make an h' to more easily construct a triangle, relative to pivot height
    private double h;

    // Projection of the arm on the ground
    private double xProjection;

    //Inputs to this Command
    DoubleSupplier extCmdFunct;
    DoubleSupplier heightCmdFunct;

    public MoveArmAtHeight(DoubleSupplier heightCmdFunct, DoubleSupplier extCmdFunct) {
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        // CONSTANTS MOVED FROM     PRE-CTOR
        armInitialLength = arm.EXTEND_MIN + arm.ARM_BASE_LENGTH + intake.WristDistToPivot + arm.L0;
        pivotHeight = arm.ARM_PIVOT_HEIGHT;
        addRequirements(arm);
        this.heightCmdFunct = heightCmdFunct;
        this.extCmdFunct = extCmdFunct;
    }

    /**
     * This only works for front side of robot...
     */

    public void execute() {
        // read inputs
        double h_cmd = heightCmdFunct.getAsDouble(); //height above floor inches
        double l_cmd = extCmdFunct.getAsDouble();    //length from from pivot

        // If target is below pivot height, used for quadrant calcs
        boolean belowPiv = h_cmd < pivotHeight;
        h = Math.abs(pivotHeight - h_cmd);           //h (inches) above or below piviot in mag
        
        //Roughly limit the extension based on game limits and robot geometry
        xProjection = MathUtil.limit(l_cmd, arm.MIN_PROJECTION, arm.MAX_PROJECTION);

        double tanRatio = (belowPiv) ? h / xProjection : xProjection / h;
        // Rotate to maintain height as projection changes
        double angle = Math.toDegrees(Math.atan(tanRatio));
        angle -= (belowPiv) ? 90.0 : 0.0;

        angle = MathUtil.limit(angle, arm.PHI_MIN, arm.PHI_MAX); 
        
        double projLen= Math.sqrt( h*h + xProjection * xProjection);    //total length of arm, from pivot point
        double ext  = projLen - (arm.ARM_BASE_LENGTH + arm.WRIST_LENGTH);   // extension required
        
        @SuppressWarnings("unused")
        double compLen = arm.getCompLen(angle);

        //limit within range, TODO: do we need to account for phi/ext interaction here?
        ext = MathUtil.limit(ext, arm.EXTEND_MIN, arm.EXTEND_MAX);

        // Extend to allow for change in projection
        arm.setExtension(ext);   //absolute ext needed projection
        arm.setAngle(angle);     //angle required for height
        /*
         * Alternative extension calculation xProjection /
         * Math.cos(arm.getAngle()) - armInitialLength
         */
    }

    public boolean isFinished() {
        return false;
    }

}