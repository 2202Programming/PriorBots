// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2023.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.base.RobotContainerOrig;
import frc.robot2023.commands.Arm.ArmMoveTo;
import frc.robot2023.commands.Arm.CollectivePositions;
import frc.robot2023.commands.Arm.ElbowMoveTo;
import frc.robot2023.commands.Arm.MoveCollectiveArm;
import frc.robot2023.subsystems.ArmSS;
import frc.robot2023.subsystems.Claw_Substyem;
import frc.robot2023.subsystems.Elbow;
import frc.robot2023.subsystems.Claw_Substyem.ClawTrackMode;

public class PlaceHighAuto extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto. */
  private Claw_Substyem claw = RobotContainerOrig.RC().claw;
  private Elbow elbow = RobotContainerOrig.RC().elbow;
  private ArmSS arm = RobotContainerOrig.RC().armSS;

  public PlaceHighAuto() {
   addCommands(
        new InstantCommand(() -> {
          claw.close();
        }),
        new InstantCommand(() -> {
          claw.setTrackElbowMode(ClawTrackMode.backSide);
        }),
        new ArmMoveTo(10.0),
        new ElbowMoveTo(70.0),
        new InstantCommand(() -> {
          claw.setTrackElbowMode(ClawTrackMode.frontSide);
        }),
        new ElbowMoveTo(155.0),
        new ArmMoveTo(37.0),
        new ElbowMoveTo(135.0),
        new InstantCommand(() -> {
          claw.close();
        }),
        new MoveCollectiveArm(CollectivePositions.power_on));

    // Use addRequirements() here to declare subsystem dependencies.
      }
}
