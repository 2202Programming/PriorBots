// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chadbot.RobotContainer;
import frc.chadbot.subsystems.Magazine_Subsystem;

public class LightGateTest extends Command {
  final Magazine_Subsystem mag;

  final String NT_Name = "/TestGates";
  final NetworkTableEntry nte_gate1;
  final NetworkTableEntry nte_gate2;
 
  /** Creates a new LightGateTest.
   * 
   * Test simply reads the values and puts them on the Network Table
   */
  public LightGateTest() {
    mag = RobotContainer.RC().magazine;

    NetworkTable table  = NetworkTableInstance.getDefault().getTable(NT_Name);
    nte_gate1 = table.getEntry("/lowerGate");
    nte_gate2 = table.getEntry("/upperGate");
    
    nte_gate1.setBoolean(true);
    nte_gate2.setBoolean(true);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        System.out.println("***********Warning: Magazine is under test - default command not running.***");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    nte_gate1.setBoolean(mag.lowerGateBlocked());
    nte_gate2.setBoolean(mag.upperGateBlocked());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("***********Magazine testing of Lightgates is complete***");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
