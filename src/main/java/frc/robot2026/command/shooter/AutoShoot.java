// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Shooter.Fuelgauge;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

public class AutoShoot extends Command {
  static boolean left_active = true;    //default to left first

  final Shooter shooter;
  final Indexer indexer;
  final Hopper hopper;
  final Fuelgauge fuelgauge;

  final double HopperNoCmdSpeed = 0.500; // was 0.706

  final DoubleSupplier speedProvider;
  final DoubleSupplier toleranceProvider;
  final double idxPct;
  final double idxLoad = 1.0; // loading speed, load to gate
  final boolean is_left;

  // state vars
  boolean gate, gate_prev; // gate edge
  boolean myturn;          // ok to shoot
  int shots_taken;

  public AutoShoot(String side, DoubleSupplier speedProvider, DoubleSupplier toleranceProvider, double idxPct) {
    // Shooter shooter, Indexer indexer, double idxPct) {
    this.shooter = RobotContainer.getSubsystem("shooter_" + side);
    this.indexer = RobotContainer.getSubsystem("indexer_" + side);
    // Targeter targeter = RobotContainer.getSubsystem(Targeter.class);
    this.hopper = RobotContainer.getSubsystem(Hopper.class);  // not on requriements
    this.fuelgauge = RobotContainer.getSubsystem(Fuelgauge.class);  // not needed on requirements

    this.speedProvider = speedProvider;
    this.toleranceProvider = toleranceProvider;
    this.idxPct = idxPct;

    this.is_left = side.startsWith("l");
    addRequirements(shooter, indexer);
    setName("AutoShoot_" + side);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shots_taken = 0;
    // gates may be false because fuel was backed off behind gate to keep flywheel
    // clear
    gate = gate_prev = indexer.hasFuel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double idxCmd = 0.0;
    shooter.flywheel.setSetpoint(speedProvider.getAsDouble() * shooter.getSpeedFactor() );
    shooter.flywheel.setVelocityTolerance(toleranceProvider.getAsDouble());
    gate = indexer.hasFuel();

    myturn = is_left  ?  AutoShoot.left_active : !AutoShoot.left_active;

    if (shooter.atSetpoint() && myturn) {
      //shoot what we have
      indexer.setPct(idxPct);
      switchShooter();          // this lets other shooter after we start, WIP 3/31/26
    } else {
      // roll indexer until we have fuel
      idxCmd = (gate) ? 0.0 : idxLoad;
      indexer.setPct(idxCmd);
    }

    // look for High->Low on gate to count shots
    if (gate_prev && (gate != gate_prev)) {
      // should be fuel leaving the bot, count it
      shots_taken++;
      indexer.setLoaded(false);
      //switchShooter();      //This switches on ball LEAVING LightGate  (used in districts)
    } //else   // testing theory on why we are waiting to shoot 
    
    if (myturn && !gate) {
      //if we don't have fuel, switch turns
      switchShooter();
    }

    // low to high indicates loading/readying a shot
    if (gate && (gate != gate_prev)) {
      indexer.setLoaded(true);
     }

    gate_prev = gate;

    if (hopper.getCurrentCommand() == null) {
      // run hopper forward if we are shooting and no hopper cmd is running
      // this is a minor violation of requirements and scheduling
      hopper.setBeltPct(HopperNoCmdSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setPct(0.0);
    shooter.addShots(shots_taken);
    shooter.flywheel.setSetpoint(0.0);
    /* NOTE: scheduling a cmd like this during Auto breaks the
           rest of the auto because of the conflict in requirements.
           Nice find JasonR,
      double spd = speedProvider.getAsDouble();
      var cmd = shooter.cmdVelocityDuration(spd, 0.300);
      CommandScheduler.getInstance().schedule(cmd);  // bad - will cancel auto running
      CommandScheduler.getInstance().schedule(new PrintCommand("Hello from AutoShoot")); // works fine
    */
    }

  @Override
  public boolean isFinished() {
    return fuelgauge.isEmpty();
    //return false;  //never ends, expect to use with a timeout or button release
  }

  void switchShooter() {
    AutoShoot.left_active = ! AutoShoot.left_active;
    myturn = false;
  }
}
