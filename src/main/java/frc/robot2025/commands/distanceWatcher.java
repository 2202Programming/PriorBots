// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.TargetWatcherCmd;
import frc.lib2202.subsystem.BaseLimelight;
import frc.lib2202.util.PoseMath;

/** Add your docs here. */
public class distanceWatcher extends TargetWatcherCmd {

    private Pose2d targetPose;
    private BaseLimelight m_Limelight = RobotContainer.getSubsystemOrNull("limelight");

    public distanceWatcher(Pose2d targetPose) {
        super();
        this.targetPose = targetPose;

    }

    @Override
    public double getTargetDistance() {
        return PoseMath.poseDistance(targetPose, m_Limelight.getBluePose());
    }

    @Override
    public void calculate() {

    }

    @Override
    public double getTargetAngle() {
        return 0.0;
    }

    @Override
    public double getTargetRPM() {
        return 0.0;
    }
}
