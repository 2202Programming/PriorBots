// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.FaceToTag;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.robot2024.Constants.Tag_Pose;
import frc.robot2024.commands.Shooter.ShooterServoSequence;
import frc.robot2024.commands.Shooter.SpeakerShooter;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.LimelightHelpers.RawFiducial;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class AutoShooting extends SequentialCommandGroup {

  final double AmpRPM = 800.0;
  final double AmpAngle = 45.0;
  
  final double TrapRPM = 1000.0;
  final double TrapAngle = 45.0;

  private Limelight limelight;
  SwerveDrivetrain drivetrain;

  /**
   * Alliance aware sequence command to face to the target and shoot the notes.
   * If not seeing tags on target, command will be canceled.
   * 
   * @param target Enum target to shoot at.
   */
  public AutoShooting(ShootingTarget target) {
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    limelight = RobotContainer.getSubsystem(Limelight.class);

    int tagID = determineTag(target);

    addCommands(new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7) );
    addCommands(new FaceToTag(tagID, 3.0));
    if (target == ShootingTarget.Speaker) {
      addCommands(new SpeakerShooter());
    } else if (target == ShootingTarget.Amp) {
      addCommands(new ShooterServoSequence(AmpAngle, AmpRPM));
    } else {
      // Trap
      addCommands(new ShooterServoSequence(TrapAngle, TrapRPM));
    }
  }

  /** Test code using speakerShooter */
  public AutoShooting(ShootingTarget target, double rpm) {
    this(target);
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    limelight = RobotContainer.getSubsystem(Limelight.class);

    int tagID = determineTag(target);

    addCommands(new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));
    addCommands(new FaceToTag(tagID, 3.0));
    if (target == ShootingTarget.Speaker) {
      addCommands(new SpeakerShooter(rpm));
    } else if (target == ShootingTarget.Amp) {
      addCommands(new ShooterServoSequence(AmpAngle, AmpRPM));
    } else {
      // Trap
      addCommands(new ShooterServoSequence(TrapAngle, TrapRPM));
    }

  }
    /**Test code */
  public AutoShooting(ShootingTarget target, double angle, double rpm) {
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    limelight = RobotContainer.getSubsystem(Limelight.class);

    int tagID = determineTag(target);

    addCommands(new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));
    addCommands(new FaceToTag(tagID, 3.0));
    if (target == ShootingTarget.Speaker) {
      addCommands(new ShooterServoSequence(angle, rpm));
    } else if (target == ShootingTarget.Amp) {
      addCommands(new ShooterServoSequence(AmpAngle, AmpRPM));
    } else {
      // Trap
      addCommands(new ShooterServoSequence(TrapAngle, TrapRPM));
    }
  }

  /**
   * Determine the tag to face based on the alliance and the target.
   * 
   * @param target operator pick from three
   * @return tagID to face
   */
  private int determineTag(ShootingTarget target) {
    // handle Optional<> from getAlliance()
    var allianceOpt = DriverStation.getAlliance();
    var alliance = DriverStation.Alliance.Blue; // default
    if (allianceOpt.isEmpty()) {
      System.out.println("Warning: FSM report no alliance, using Blue.");
    } else {
      // use what we are given
      alliance = allianceOpt.get();
    }

    if (alliance == DriverStation.Alliance.Blue) {
      // on Blue Alliance
      if (target == ShootingTarget.Speaker) {
        return 7;
      } else if (target == ShootingTarget.Amp) {
        return 6;
      } else {

        RawFiducial[] tags = limelight.getAprilTagsFromHelper();
        for (RawFiducial tag : tags) {
          if (tag.id == 14) {
            return 14;
          } else if (tag.id == 15) {
            return 15;
          } else if (tag.id == 16) {
            return 16;
          }
        }
      }
    } else {
      // on Red Alliance
      if (target == ShootingTarget.Speaker) {
        return 4;
      } else if (target == ShootingTarget.Amp) {
        return 5;
      } else {
        RawFiducial[] tags = limelight.getAprilTagsFromHelper();
        for (RawFiducial tag : tags) {
          if (tag.id == 11) {
            return 11;
          } else if (tag.id == 12) {
            return 12;
          } else if (tag.id == 13) {
            return 13;
          }
        }
      }
    }
    System.out.println("Invalid in AutoShooting");
    return 17;// DNE
  }

  /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  @SuppressWarnings("unused")
  private boolean checkForTarget(double tagID) {
    RawFiducial[] tags = limelight.getAprilTagsFromHelper();
    for (RawFiducial tag : tags) {
      if (tag.id == tagID) {
        return true;
      }
    }
    return false;
  }

  public enum ShootingTarget {
    Speaker,
    Amp,
    Trap
  }
}