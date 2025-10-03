// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.test;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot2024.subsystems.sensors.Sensors_Subsystem;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class getTrajectoryFollowTest extends Command {
  final Sensors_Subsystem sensors;
  final SwerveDrivetrain drivetrain;
  final OdometryInterface odo;
  Set<Subsystem> requirements;
  Command work;

  /** Creates a new getTrajectoryFollowTest. 
   * @return */
  public  getTrajectoryFollowTest(Sensors_Subsystem ns, SwerveDrivetrain drivetrain) {
    sensors = ns;
    this.drivetrain = drivetrain;
    this.odo = RobotContainer.getSubsystem("odometry");
    requirements = new  HashSet<Subsystem>();
    requirements.add(drivetrain);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  // An example trajectory to follow. All units in feet.
    Rotation2d current_angle = new Rotation2d(sensors.getYaw());
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, current_angle),
        List.of(
        // new Translation2d(0.0, 0.25),
        // new Translation2d(0.0, 0.5),
        // new Translation2d(0.0, 0.75)
        ),
        new Pose2d(0, 3.0, current_angle),
        new TrajectoryConfig(2.0, 0.5) ); // max velocity, max accel
  
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        odo::getPose, // Functional interface to feed supplier
        drivetrain.getKinematics(),
        // Position controllers
        new PIDController(4.0, 0.0, 0.0),
        new PIDController(4.0, 0.0, 0.0),
        new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(.3, .3)),
        // Here, our rotation profile constraints were a max velocity
        // of 1 rotation per second and a max acceleration of 180 degrees
        // per second squared
        drivetrain::drive,
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    odo.setPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    work = swerveControllerCommand.andThen(() -> drivetrain.stop()).withTimeout(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    work.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    work.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return work.isFinished();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }

}
