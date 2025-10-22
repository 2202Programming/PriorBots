package frc.robot2025.commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.LimelightV1;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.UX.TrimTables.Trim;
import frc.robot2025.Constants.TheField;

public class DriveToPickupTag extends Command{
    static String TrimTableName = "DriveToPickup";

    static double LeftOffset = 0.0;    //[m]
    static double RightOffset = 0.0;   //[m]
    static double BackupOffset = 0.48; //[m]

    static PathConstraints constraints = new PathConstraints(2.8, 1.8, Math.PI, Math.PI / 2.0);

    final LimelightV1 LL;
    final String LLName;
    final OdometryInterface odo;
    final String odoName = "vision_odo";   //todo make an arg
    final int tagIdx;
    final Trim redBackoffTrim;
    final Trim blueBackoffTrim;
    final Trim redXyTrim;
    final Trim blueXyTrim;
    final double xyOffset;
    final String side;
    
    // command vars set at init time
    boolean done;
    Pose2d target;
    Command moveComand;
    public DriveToPickupTag(String side){
        odo = RobotContainer.getSubsystemOrNull(odoName);
        LL = RobotContainer.getObjectOrNull("limelight");
        LLName = (LL != null) ? LL.getName() : "no-ll-found";  //name if we need to use LLHelpers directly
        this.side = side.toLowerCase();

        // pick a direction to go, left , right in TheField
        tagIdx = this.side.startsWith("l") ? 1 : 0; 
        xyOffset = this.side.startsWith("l") ? LeftOffset : RightOffset;

        // setup trims for both red/blue
        redBackoffTrim = new Trim(TrimTableName, "Pickup_backoff_Red_" + side, 0.0);
        blueBackoffTrim = new Trim(TrimTableName, "Pickup_backoff_Blue_" + side, 0.0);
        redXyTrim = new Trim(TrimTableName, "Pickup_shift_Red_" + side, 0.0);
        blueXyTrim = new Trim(TrimTableName, "Pickup_shift_Blue_" + side, 0.0);
    }

    @Override
    public void initialize() {
        Trim xyTrim;
        Trim backoffTrim;
        done = true;
        //protect from missng required ss
        if (LL == null) return;
        if (odo == null) return;
        moveComand = null;       

        Alliance alliance;
        // get our alliance
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();            
        }
        else return; // no alliance, bail

        // set trims based on our color
        if (alliance == Alliance.Blue) {
            xyTrim = blueXyTrim;
            backoffTrim = blueBackoffTrim;
        } else {
            xyTrim = redXyTrim;
            backoffTrim = redBackoffTrim;
        }
        
        // set LL targets to our reef only
        int tagId = (alliance == Alliance.Blue) ? TheField.PickupIdsBlue[tagIdx] : TheField.PickupIdsRed[tagIdx];
        var p3d = TheField.fieldLayout.getTagPose(tagId).get();

        var rot2d = p3d.getRotation().toRotation2d();//.getAngle();
        // left/right shift - rotate tag vector 90 outbound, use UXTrim
        var lrRot = rot2d.plus( side.startsWith("l") ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
        double lr_dx = lrRot.getCos() * xyTrim.getValue(xyOffset);
        double lr_dy = lrRot.getSin() * xyTrim.getValue(xyOffset);

        // Backup robot along tag face, use trims to adjust
        double dx = rot2d.getCos() * backoffTrim.getValue(BackupOffset);
        double dy = rot2d.getSin() * backoffTrim.getValue(BackupOffset);

        //rotate tag vector to calc L/R shift
        target = new Pose2d(p3d.getX() + dx + lr_dx, p3d.getY() + dy + lr_dy, rot2d);

        moveComand = new MoveToPose(odoName, constraints, target);
        if (moveComand != null) {
            moveComand.initialize();
        }
        done = false;        
    }

    @Override
    public void execute() {
        if (moveComand == null) return;
        moveComand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (moveComand != null) {
            moveComand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if (moveComand != null) {
            done = moveComand.isFinished();
        }
        return done;
    }

}
