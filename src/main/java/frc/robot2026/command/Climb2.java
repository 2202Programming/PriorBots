package frc.robot2026.command;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.Constants.TheField;
import frc.robot2026.command.swerve.ConstantVelocity;
import frc.robot2026.subsystems.Climber;

public class Climb2 extends Command {

    //Hardware
    final Climber climber;
    final DriveTrainInterface sdt;
    final PathConstraints constraints;
    final String vision_name = "vision_odo";

    // POSITION CONSTANTS   
    final double TW = 0.90;     // TOWER WIDTH
    final double TL = 1.00;     // TOWER LENGTH
    final double BW = 0.86;     // BUMPER WIDTH
    final double BL = 0.81;     // BUMPER LENGTH

    // final move
    final double MOVE_VEL = -0.2;     // [m/s] Robot Coordinates, moves backwards
    final double TIME_Vx = 0.5;  // [s]

    //Blue Side Transform - not including tag coords
    final Transform2d BluePoseLeft = new Transform2d(
        new Translation2d(TL - BL/2.0, TW/2.0 + BW/2.0), Rotation2d.k180deg);
    final Transform2d BluePoseRight = new Transform2d(
        new Translation2d(TL + BL/2.0, -TW/2.0 - BW/2.0), Rotation2d.kZero);
    final int BlueTagID = 31;
    final Pose2d BlueTag;

    // Red Side - not includeing tag coords
    final Transform2d RedPoseLeft = new Transform2d(
        new Translation2d(-TL + BL/2.0, -TW/2.0 - BW/2.0), Rotation2d.k180deg);
    final Transform2d RedPoseRight = new Transform2d(
        new Translation2d(-TL - BL/2.0, TW/2.0 + BW/2.0), Rotation2d.kZero);
    final int RedTagID = 15;
    final Pose2d RedTag;

    // we expect left side to work better because limelight is facing tags

    final boolean isLeft;
    final Transform2d  redOffset;   // based on Left or right choice
    final Transform2d  blueOffset;  // based on Left or right choice   
    final Pose2d redGoal;
    final Pose2d blueGoal;

    Pose2d goal;  // computed during init based on red/blue
    SequentialCommandGroup cmd;  // built in init, run here

    public Climb2(char side) {
        this.isLeft =  side == 'L' || side == 'l';
        setName("Climb2-" + side);
        //select left/right offsets
        redOffset  = (isLeft) ? RedPoseLeft : RedPoseRight;
        blueOffset = (isLeft) ? BluePoseLeft : BluePoseRight;

        //target tags
        BlueTag = TheField.fieldLayout.getTagPose(BlueTagID).get().toPose2d();
        RedTag = TheField.fieldLayout.getTagPose(RedTagID).get().toPose2d();

        //define goals for red or blue by adding our offsets
        redGoal = RedTag.plus(redOffset);
        blueGoal = BlueTag.plus(blueOffset);

        // robot hardware subsystems
        climber = RobotContainer.getSubsystem("climber");
        sdt = RobotContainer.getSubsystem("drivetrain");
        RobotLimits limits = RobotContainer.getRobotSpecs().getRobotLimits();
        constraints = new PathConstraints(limits.kMaxSpeed, limits.kMaxSpeed / 1.33,
            limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / 0.75);  
        
        // no requirements, we will add them to the cmd we build in initialize   
    }

     @Override
      public void initialize() {
        goal = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueGoal : redGoal;

        // now build a command to get to the goal and climb - hack, use moveTo so must be 1m away
        cmd = new SequentialCommandGroup();
        cmd.addCommands(
            new PrintCommand("climb2 goal -> "+ goal.toString()),
            new MoveToPose(vision_name, constraints, goal),     // should in-line with post when done, 
            climber.armsToPoint(Climber.ExtendPosition).withTimeout(2.0),
            new ConstantVelocity(MOVE_VEL, Rotation2d.kZero, TIME_Vx),   // zero roatation, moving in robot coords
            climber.armsToPoint(Climber.ClimbPositon)            
        );
        
        cmd.initialize();
    }

    @Override
    public void execute() {
        cmd.execute();
    }
    
    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }

    public void end(boolean interrupted) {
        cmd.end(interrupted);
    }

}
