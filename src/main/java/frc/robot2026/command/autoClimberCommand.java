// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.command.swerve.RotateTo;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.util.PoseMath;
import frc.robot2026.Constants.TheField;

import frc.robot2026.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoClimberCommand extends Command {
  // SequentialCommandGroup {
  /** Creates a new autoClimberCommand. */
  final Climber climber;
  final DriveTrainInterface sdt;
  final RobotLimits limits;
  final PathConstraints constraints;

  // debugging use only
  final OdometryInterface odo;

  final Pose3d blueCenter;
  final Pose3d redCenter;
  final boolean leftSide;
  // computed in init based on Alliance and side...
  Pose2d realCenter;

  // Assuming our bot is symmetric
  final static double BW = 0.86;
  final static double BL = 0.81;

  final static double TL = 1.095; //tower length from tag to front of tower
  final static double TW = 0.895;

  public autoClimberCommand(boolean leftSide) {
    this.leftSide = leftSide;
    Optional<Pose3d> BlueCenter = TheField.fieldLayout.getTagPose(31); // If the field exists, give us the red and blue
                                                                       // position
    Optional<Pose3d> RedCenter = TheField.fieldLayout.getTagPose(15);
    blueCenter = BlueCenter.isPresent() ? BlueCenter.get() : null;
    redCenter = RedCenter.isPresent() ? RedCenter.get() : null;
    limits = RobotContainer.getRobotSpecs().getRobotLimits();

    odo = RobotContainer.getSubsystem("vision_odo");
    climber = RobotContainer.getSubsystem("climber");
    sdt = RobotContainer.getSubsystem("drivetrain");
    constraints = new PathConstraints(1.0, limits.kMaxSpeed / 1.33,
        limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / 0.75);

    // no requirements, we will add them to the cmd we build in initialize
  }

  @Override
  public void initialize() {
    realCenter = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueCenter.toPose2d() : redCenter.toPose2d();
    Pose2d currentPose = odo.getPose();
    Pose2d pose;
    Rotation2d targetRot;
    targetRot = (!leftSide) ? (realCenter.getRotation())
        : realCenter.getRotation().plus(Rotation2d.fromDegrees(180.0));
    pose = (leftSide) ? // Gives us our tranformation based on left or right side
        realCenter.transformBy(new Transform2d(
                new Translation2d(TL - BL * 0.5 - climberManuver.clof,    //x
                (BW + TW) *.5),                                           //y
                Rotation2d.fromDegrees(180.0)))                   //
        : realCenter.transformBy(new Transform2d(
                new Translation2d(TL + BL * 0.5 + climberManuver.clof,
                                  -0.5 * (BW + TW)), 
                                   Rotation2d.fromDegrees(0.0)));

    var cmd = new SequentialCommandGroup(
        new PrintCommand("climb pose " + pose.toString() + " dist=" + PoseMath.poseDistance(currentPose, pose)),
        ((0.5 < Math.abs((odo.getPose().getRotation().minus(targetRot).getDegrees())))
            ? ((targetRot.getDegrees() == 180.0)
                ? (new RotateTo(new Translation2d(odo.getPose().getX() - 1.0, odo.getPose().getY()),
                    new Translation2d(odo.getPose().getX() - 1.0, odo.getPose().getY())))
                : (new RotateTo(new Translation2d(odo.getPose().getX() + 1.0, odo.getPose().getY()),
                    new Translation2d(odo.getPose().getX() + 1.0, odo.getPose().getY()))))
            : new PrintCommand("At Rotation")),
        ((dontCrashTM() || ((PoseMath.poseDistance(currentPose, pose) > 1.0))) ? // If we are on the wrong side OR we
                                                                                 // are greater than a meter away
            new MoveToPose("vision_odo", constraints, pose)
            : new PrintCommand("Climber is close enough")),
        climber.armsToPoint(Climber.ExtendPosition), //.withTimeout(2.0),
        //new WaitCommand(2.0),
        new climberManuver(leftSide),
        climber.armsToPoint(Climber.ClimbPositon));//.withTimeout(2.0));

    cmd.addRequirements(climber, sdt);
    cmd.setName("autoClimb-" + pose.toString());
    cmd = cmd.andThen(new PrintCommand("autoClimb DONE!!"));

    // run what we built
    CommandScheduler.getInstance().schedule(cmd);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  private boolean dontCrashTM() { // TODO check if this actually works
    // Checks if we will run into the tower when using waypoints. May change in
    // general
    double LDC; // Line Dont Cross
    double currentPoseY = odo.getPose().getY();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      if (leftSide) {
        LDC = blueCenter.getY() + (BW * .5 + .45);
      } else {
        LDC = blueCenter.getY() - (BW * .5 + .45);
        currentPoseY = -currentPoseY;
        LDC = -LDC;
      }
    } else {
      if (leftSide) {
        LDC = redCenter.getY() - (BW * .5 + .45);
        currentPoseY = -currentPoseY;
        LDC = -LDC;
      } else {
        LDC = redCenter.getY() + (BW * .5 + .45);
      }
    }
    return currentPoseY > LDC;
  }

}
