// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.util;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot2026.Constants;

public class BetterAutoChooser {
    private static final boolean RIGHT_IS_FLIPPED = false;
    private static final String DEFAULT_NAME = "None";
    private static final String FLIP_PREFIX = "FLIP ";

    private static final double TRANSLATION_ERROR = 0.5; // meters
    private static final double ROTATION_ERROR = 0.1; // rotations

    private static Map<String, Pose2d> startingPoses = new HashMap<>();

    private static SendableChooser<Command> chooser;

    public static SendableChooser<Command> buildAutoChooser() {
        chooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", chooser);
        chooser.setDefaultOption(DEFAULT_NAME, Commands.none());
        if (!AutoBuilder.isConfigured()) {
            return chooser;
        }

        for (String name : AutoBuilder.getAllAutoNames()) {
            if (name.startsWith(FLIP_PREFIX)) {
                // Make the flipped paths
                String remainder = name.substring(FLIP_PREFIX.length());
                registerAuto("Original " + remainder + " (Generated)", new PathPlannerAuto(name, RIGHT_IS_FLIPPED));
                registerAuto("Flipped " + remainder + " (Generated)", new PathPlannerAuto(name, !RIGHT_IS_FLIPPED));
            } else {
                registerAuto(name, new PathPlannerAuto(name));
            }
        }

        return chooser;
    }

    private static void registerAuto(String name, PathPlannerAuto auto) {
        chooser.addOption(name, auto.withName(name));
        Pose2d startingPose = auto.getStartingPose();
        if (startingPose != null) {
            startingPoses.put(name, startingPose);
        }
    }

    public static boolean checkPose(String autoName, Pose2d robotPose) {
        if (!startingPoses.containsKey(autoName)) {
            return false;
        }
        Pose2d start = startingPoses.get(autoName);
        Transform2d offset = allianceRelativeFlip(robotPose).minus(start);
        double xyError = offset.getTranslation().getNorm();
        double rotError = Math.abs(offset.getRotation().getRotations());
        return (xyError <= TRANSLATION_ERROR) && (rotError <= ROTATION_ERROR);
    }

    public static Pose3d allianceRelativeFlip(Pose3d pose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      return new Pose3d(
          Constants.FieldConstants.kFieldLength.minus(pose.getMeasureX()),
          Constants.FieldConstants.kFieldWidth.minus(pose.getMeasureY()),
          pose.getMeasureZ(),
          pose.getRotation().rotateBy(new Rotation3d(Rotation2d.kPi)));
    } else {
      return pose;
    }
  }

    public static Pose2d allianceRelativeFlip(Pose2d pose) {
        return allianceRelativeFlip(new Pose3d(pose)).toPose2d();
    }
}
