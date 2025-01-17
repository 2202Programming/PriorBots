// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

public class TestShoot extends Command {
  final double TESTANGLE = 0.0;
  final double TESTTOL = 0.02;

  Shooter_Subsystem shooter;

  NetworkTable table;
  NetworkTableEntry ntUpperRPM;   //FW speeds (output)
  NetworkTableEntry ntLowerRPM;
  NetworkTableEntry ntBallVel;    // ball physics (input) 
  NetworkTableEntry ntBallRPS;
  
  ShooterSettings  cmdSS;         // instance the shooter sees
  ShooterSettings  prevSS;        // instance for prev State

  /** Creates a new TestShoot. */
  public TestShoot(Shooter_Subsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  
    table = NetworkTableInstance.getDefault().getTable("TestShooter");
    ntUpperRPM = table.getEntry("UpperRPM");
    ntLowerRPM = table.getEntry("LowerRPM");
    ntBallVel = table.getEntry("BallVel");
    ntBallRPS = table.getEntry("BallRPS");

    ntUpperRPM.setDouble(0);
    ntLowerRPM.setDouble(0);
    ntBallVel.setDouble(0);
    ntBallRPS.setDouble(0);
    cmdSS = new ShooterSettings(ntBallVel.getDouble(0.0), ntBallRPS.getDouble(0.0), TESTANGLE, TESTTOL);
    prevSS = new ShooterSettings(cmdSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spinup(cmdSS);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //read network for new cmd values
    cmdSS.vel = ntBallVel.getDouble(cmdSS.vel);
    cmdSS.rps = ntBallRPS.getDouble(cmdSS.rps);
    
    if (!cmdSS.equals(prevSS)) {
      shooter.spinup(cmdSS);
      prevSS.vel = cmdSS.vel;
      prevSS.rps = cmdSS.rps;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.off();
  }
}
