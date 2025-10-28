// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.BlinkyLights;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.ShooterServo;
import frc.robot2024.subsystems.Transfer;
import frc.lib2202.subsystem.BlinkyLights.BlinkyLightUser;

/*
 * An example of how to use the blinky lights in your command.
 * 
 * initialize()  - call enablelights()
 * 
 * @overide the colorProvider function with something meaningful for your command
 * colorProvider() - returns the Color8Bit you want
 * 
 * That's it!
 */
public class BlinkyLightNoteColorCmd extends BlinkyLightUser {
  /** Creates a new Lights Command */
  private Color8Bit myColor = BlinkyLights.RED;
  final ShooterServo shooter;
  final Transfer transfer;
  final Intake intake;


  public BlinkyLightNoteColorCmd() {
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    transfer = RobotContainer.getSubsystem(Transfer.class);
    intake = RobotContainer.getSubsystem(Intake.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    enableLights();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myColor = BlinkyLights.BLACK;    
    if(DriverStation.getMatchTime() < 15.0){
      myColor = BlinkyLights.ORANGE;
    }
    else if(intake.senseNote()){
      myColor = BlinkyLights.YELLOW;
    }
   else if(transfer.hasNote()){
    myColor = BlinkyLights.GREEN; 
   }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //required by blinkylightuser if you want the lights to change
  @Override
  public Color8Bit colorProvider() {
    return myColor;
  }


}
