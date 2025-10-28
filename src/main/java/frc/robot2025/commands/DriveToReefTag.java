package frc.robot2025.commands;
import static frc.lib2202.Constants.DEGperRAD;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.ILimelight;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.util.ModMath;
import frc.robot2025.Constants.TheField;


public class DriveToReefTag extends Command { 
    //Robot left/right offsets for aligning with reef
    static double LeftOffset =  -0.11;  //[m]   was -0.09 4/3/25
    static double RightOffset = -0.42;  //[m]
    static double BackupOffset = 0.48;  //[m]   was .50
    static Rotation2d LLRot = Rotation2d.k180deg;

    static Map<Integer, Pose2d> blueReefLeft = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> blueReefRight = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> redReefLeft = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> redReefRight = new HashMap<Integer, Pose2d>();

    static PathConstraints constraints = new PathConstraints(2.5,1.75, Math.PI, Math.PI / 2.0);

    static void buildPositions(Map<Integer, Pose2d> map, int[] tags, double l_offset, double r_offset, boolean isLeft, boolean isRed) {
        // loop over given tags and build the 2d targets
        double x, y, rot;
        for (int tagId : tags) {
            Pose3d tagPose = TheField.fieldLayout.getTagPose(tagId).get();           
            x = tagPose.getX();
            y = tagPose.getY();
            rot = tagPose.getRotation().getAngle();
            var rotdeg = rot*DEGperRAD; //debug assist
            var rotdegmod = ModMath.fmod360_2(rotdeg);
            var rot2d = new Rotation2d(rot);

            // Backup robot along tag face
            double dx = rot2d.getCos()*BackupOffset;
            double dy = rot2d.getSin()*BackupOffset;

            // rotate based on side of reef
            boolean rotDirFlip = (Math.abs(rotdegmod) >=90.0);           

             //use correct driver perspective by alliance
            rotDirFlip = (isRed) ? !rotDirFlip : rotDirFlip;

            // figure out why way to shift for reef pole
            double lr_offset = (isLeft) ?  l_offset : r_offset;
           
            // adjust L/R - rotate based on which side the tags are on
            var lrRot =(rotDirFlip) ? 
                rot2d.plus(Rotation2d.kCW_90deg) : 
                rot2d.plus(Rotation2d.kCW_90deg); // was CCW
            double lr_dx = lrRot.getCos()*lr_offset;
            double lr_dy = lrRot.getSin()*lr_offset;

            Pose2d targetPose = new Pose2d(x +dx + lr_dx,
                                           y + dy + lr_dy, rot2d.plus(LLRot));
            
            map.put(tagId, targetPose);            
        }
    }

    // setup our targets
    static {
        buildPositions(blueReefLeft,  TheField.ReefIdsBlue, LeftOffset, RightOffset, true, false);
        buildPositions(blueReefRight, TheField.ReefIdsBlue, LeftOffset, RightOffset, false, false);
        buildPositions(redReefLeft,   TheField.ReefIdsRed, LeftOffset, RightOffset, true, true);
        buildPositions(redReefRight,  TheField.ReefIdsRed, LeftOffset, RightOffset, false, true);
    }
    
    final boolean leftSide;  //side of reef to deliver to
    final Map<Integer, Pose2d> redPoses;
    final Map<Integer, Pose2d> bluePoses;
    final ILimelight LL;
    final String LLName;
    final OdometryInterface odo;
    final String odoName = "vision_odo";   //todo make an arg
    final int no_vision_idx;

    // command vars set at init time
    boolean failedAtInit;
    Command moveComand;
    Map<Integer, Pose2d> alliancePoses;
    double TA_MIN = 0.28;
    int foundTag;
    int last_usedTag;
    Pose2d targetPose;
    Pose2d last_targetPose = null;
    int[] targetTags;
    int[] orderedTags;

    //on Distance schedule
    double schedDistance = 0.0;
    Command schedCommand = null;
    boolean schedRunning = false;
    boolean schedOnce = false;

    public DriveToReefTag(String reefSide) {
        this(reefSide, -1);
    }

    public DriveToReefTag(String reefSide, int indexTag) {
        // convert 1 ..6 to 0..5 or -1 if no target
        no_vision_idx = (indexTag > 0 && indexTag <= 6) ? indexTag - 1 : -1;
        odo = RobotContainer.getSubsystemOrNull(odoName);
        LL = RobotContainer.getObjectOrNull("limelight");
        LLName = (LL != null) ? LL.getLLName() : "no-ll-found";  //name if we need to use LLHelpers directly

        // pick a direction to go
        leftSide = reefSide.toLowerCase().startsWith("l");

        // setup red/blue for this commands side
        redPoses = leftSide ? redReefLeft : redReefRight;
        bluePoses = leftSide ? blueReefLeft : blueReefRight;

        SimTesting.initSimTesting();
    }
    
    @Override
    public void initialize() {
        schedRunning = false;
        schedOnce = false;
        failedAtInit = true;
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

        failedAtInit = false; 

         // set LL targets to our reef only
        alliancePoses = (alliance == Alliance.Blue) ? bluePoses : redPoses;
        targetTags = keysToInt(alliancePoses);
        LL.setTargetTags(targetTags);

        //these are in driver order for no_vision_idx
        orderedTags = (alliance == Alliance.Blue) ? TheField.ReefIdsBlue : TheField.ReefIdsRed;

        // check to see if we are close (d < .50m) to last completed/found tag
        // so if we can't see a tag and are close, just compute path
        // TODO - PP won't run a path closer than 0.5, so this isn't working 3/21/25
        var dist = (last_targetPose != null) ? odo.getDistanceToTranslation(last_targetPose.getTranslation()) : 99.0;
        foundTag = (!LimelightHelpers.getTV(LLName) && dist < 0.50) ?  
            last_usedTag :  //close, use last completed
            0;              //wait for limelight
    }

    @Override
    public void execute() { 
        // just waiting for our move to finish, no need to look for tag.
        if (moveComand != null) {
            moveComand.execute();  //run our moveCommand

            if (schedCommand != null && schedRunning) {
                schedCommand.execute();
            }
            double distanceToTarget = odo.getDistanceToTranslation(targetPose.getTranslation());
            // run our schedCommand if hasn't been done and we are close
            if (schedCommand != null && !schedRunning && !schedOnce &&
                distanceToTarget <= schedDistance) {
                schedCommand.initialize();
                schedRunning = true;
                schedOnce = true;
            }
            return;
        }

        // Look for our tags and create a moveTo if we find a quality tag       
        if (LimelightHelpers.getTV(LLName) ){
             // read LL for tag
            foundTag = (int)LimelightHelpers.getFiducialID(LLName);
        }

        if (RobotBase.isSimulation()) {
            foundTag = SimTesting.simGetTarget();
        }

        // skip vision location if we were given an index
        if (no_vision_idx >=0 ) {
            foundTag = orderedTags[no_vision_idx];
        }
        
        // found a tag in our set, nearest I hope, build a path
        if (foundTag > 0 ) {
            targetPose = alliancePoses.get(foundTag);
            // build path to target
            if (targetPose == null) return;   // not a reef target on our side

            moveComand = new MoveToPose(odoName, constraints, targetPose);

            // now run the moveCommand in this command's context
            if (moveComand != null) {                
                moveComand.initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (moveComand != null) {
            moveComand.end(interrupted);
        }
        if (schedCommand != null && schedRunning) {
            schedCommand.end(interrupted);
        }
        // keep last foundTag for shortpath WIP
        last_usedTag = foundTag;
        last_targetPose = targetPose;
        //restore or normal tag list.
        LL.resetTargetTags();
    }

    @Override
    public boolean isFinished() {
        if (failedAtInit) {
            // If our preflight checks failed (or in the middle of init)
            return true;
        }

        // If there is no move command, we assume finished
        boolean move_done = true;
        if (moveComand != null) {
            move_done = moveComand.isFinished();
        }

        boolean schedCmd_done;
        if (schedCommand == null){
            // No scheduled command exists, therefore nothing to do
            schedCmd_done = true;
        }else if (schedRunning){
            // the command is running, let's check its status
            schedCmd_done = schedCommand.isFinished();
            if (schedCmd_done) {
                // We are responsible for ending our child command if it is finished
                schedCommand.end(false);
                schedRunning = false;
            }
        }else if (schedOnce){
            // The command exists and was run once but is not running, therefore it is done running
            schedCmd_done = true;
        }else {
            // If we are here, the command exists and is not currently running and not done.
            // Therefore it is not started

            // If the move has finished, but the command has not been scheduled, then the command
            // will never start. Set command to done and finish. IE give up.
            schedCmd_done = move_done; 
        }

        // We are responsible for both commands, we are not done until they both are
        return move_done && schedCmd_done;
    }

    public DriveToReefTag withDistanceScheduleCmd(Command cmd, double distance) {
        schedDistance = distance;
        schedCommand = cmd;
        return this;
    }

    int[] keysToInt(Map<Integer, Pose2d> map) {
        var keys = map.keySet();
        int[] ints = new int[keys.size()];
        int i=0;
        for (int v : keys){
            ints[i++] = v;
        }
        return ints;
    }

    static final class SimTesting  {
        static NetworkTable table;
        static NetworkTableEntry nt_lltag;

        static void initSimTesting() {
            table = NetworkTableInstance.getDefault().getTable("Test/CmdMoveToReef");
            nt_lltag = table.getEntry("LLTag");
            nt_lltag.setInteger(-1);
        }

        static int simGetTarget() {
            return (int)nt_lltag.getInteger(-1);
        } 

    }

}
