// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chadbot.Constants.NTStrings;
import frc.chadbot.commands.Shoot.SolutionProvider;
import frc.chadbot.subsystems.Sensors_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class DriveControllerDrivetrain extends Command implements SolutionProvider {

  public enum DriveModes {
    robotCentric("Robot Centric"),
    fieldCentric("Field Centric"),
    hubCentric("Hub Centric"),
    intakeCentric("Intake Centric");
    private String name;
    private DriveModes(String name) {
      this.name = name;
    }
    public String toString() {
      return name;
    }
  }

  //subsystems
  SwerveDrivetrain drivetrain;
  HID_Xbox_Subsystem dc;
  Shooter_Subsystem shooter;
  Limelight limelight;
  Sensors_Subsystem sensors;

  //commands
  RobotCentricDrive m_robotCentricDrive;
  FieldCentricDrive m_fieldCentricDrive;
  HubCentricDrive m_hubCentricDrive;
  IntakeCentricDrive m_intakeCentricDrive;

  DriveCmdClass currentCmd;
  DriveModes requestedDriveMode = DriveModes.fieldCentric;
  DriveModes currentDriveMode = DriveModes.fieldCentric;
  DriveModes lastDriveMode = DriveModes.fieldCentric;
  boolean currentlyShooting = false;
  boolean shootingRequested = false;
  boolean hasSolution = false;
  double shoot_on_time = -1.0;    // added post season for Shooting without LL target

  NetworkTable table;
  NetworkTable shooterTable;
  NetworkTable positionTable;
  NetworkTableEntry driveMode;
  public final String NT_Name = "DC"; 
  public final String NT_ShooterName = "Shooter"; 

  NetworkTableEntry nt_roll_factor;
  NetworkTableEntry nt_pitch_factor;

  double roll_factor = 0;
  double pitch_factor = 0;

  int log_counter = 0;
  boolean tip_correction_mode = false;

  PIDController tipRollPid;
  double roll_kP = 0.3;
  double roll_kI = 0.0;
  double roll_kD = 0.0;
  double tipRollPidOutput = 0.0;

  PIDController tipPitchPid;
  double pitch_kP = 0.3;
  double pitch_kI = 0.0;
  double pitch_kD = 0.0;
  double tipPitchPidOutput = 0.0;

  double requested_pitch_P = pitch_kP;
  double requested_pitch_I = pitch_kI;
  double requested_pitch_D = pitch_kD;

  double requested_roll_P = roll_kP;
  double requested_roll_I = roll_kI;
  double requested_roll_D = roll_kD;

  public DriveControllerDrivetrain()  {
    this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    this.dc = RobotContainer.getSubsystem(HID_Xbox_Subsystem.class);
    this.limelight = RobotContainer.getSubsystem(Limelight.class);
    this.sensors = RobotContainer.getSubsystem(Sensors_Subsystem.class);

    m_robotCentricDrive = new RobotCentricDrive();
    m_fieldCentricDrive = new FieldCentricDrive();
    m_hubCentricDrive = new HubCentricDrive(drivetrain, dc, limelight);
    m_intakeCentricDrive = new IntakeCentricDrive(drivetrain, dc);

    tipRollPid = new PIDController(roll_kP, roll_kI, roll_kD);
    tipPitchPid = new PIDController(pitch_kP, pitch_kI, pitch_kD);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    positionTable = NetworkTableInstance.getDefault().getTable(NTStrings.NT_Name_Position);
    driveMode = table.getEntry("/DriveController/driveMode");
    //NThasSolution = shooterTable.getEntry("/DriveController/HasSolution");

    nt_roll_factor = table.getEntry("/DriveController/RollFactor");
    nt_pitch_factor = table.getEntry("/DriveController/PitchFactor");

    SmartDashboard.putNumber("Requested Pitch P", requested_pitch_P);
    SmartDashboard.putNumber("Requested Pitch I", requested_pitch_I);
    SmartDashboard.putNumber("Requested Pitch D", requested_pitch_D);
    SmartDashboard.putNumber("Requested Roll P", requested_roll_P);
    SmartDashboard.putNumber("Requested Roll I", requested_roll_I);
    SmartDashboard.putNumber("Requested Roll D", requested_roll_D);
  }

  @Override
  public void initialize() {
    currentCmd = m_fieldCentricDrive;
    CommandScheduler.getInstance().schedule(currentCmd); // start default drive mode
  }

  @Override
  public void execute() {
    checkDropout();
    checkRequests();
    updateNT();
    //checkPid();
    checkTip();
  }

  /**
   * isOnTarget provides feedback to shoot command being run.
   */
  @Override
  public boolean isOnTarget(){
    // free shoot mode - just return true 

    // Post season - might just have one controller, and could be hub-centric without a LL target
    // so check shootwithoutTarget() which just delays 250ms 
    if (currentDriveMode == DriveModes.hubCentric)
      return shootWithoutTarget(250.0) || m_hubCentricDrive.isReady();   //have target or waited 250 ms
    return true;   //all other modes driver is the targeting solution
  }

  void checkDropout(){
    if ((Math.abs(dc.getXYRotation())>0.1) && (currentDriveMode==DriveModes.intakeCentric)){
      //driver is trying to rotate while in intakeCentric, drop out of intake mode
      requestedDriveMode = DriveModes.fieldCentric;
    }
  }

  void checkRequests(){
    if (requestedDriveMode != currentDriveMode){ //new drive mode requested
      lastDriveMode = currentDriveMode;
      currentDriveMode = requestedDriveMode;
      currentCmd.end(true); //stop prior command
      switch (currentDriveMode){
        case robotCentric:
          currentCmd = m_robotCentricDrive;
          break;
  
        case fieldCentric:
          currentCmd = m_fieldCentricDrive;
          break;    

        case hubCentric:
          currentCmd = m_hubCentricDrive;
          break;      
        
        case intakeCentric:
          currentCmd = m_intakeCentricDrive;
          break;
      }
      CommandScheduler.getInstance().schedule(currentCmd);
    }
  }

  public void cycleDriveMode() {
    //Current use case is only to allow toggling between field and intake centric
    //Make sure if in hubcentric (trigger held) that toggling drops back to default fieldcentric
    switch (currentDriveMode) {
      case robotCentric:
      case hubCentric:
      case fieldCentric:
        requestedDriveMode = DriveModes.intakeCentric;
        break;

      case intakeCentric:
        requestedDriveMode = DriveModes.fieldCentric;
        break;
    }
  }

  public void setRobotCentric() {
    requestedDriveMode = DriveModes.robotCentric;
  }

  public void setFieldCentric() {
    requestedDriveMode = DriveModes.fieldCentric;
  }

  public void turnOnShootingMode(){
    shootingRequested = true;
    limelight.enableLED();

    // post season, we may not have LL but still want to shoot
    shoot_on_time = RobotController.getFPGATime() / 1000.0;

  }

  public void turnOffShootingMode(){
    shootingRequested = false;
    shoot_on_time = -1.0;
  }

  @Override
  public void end(boolean interrupted) {
    currentCmd.end(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
      // update network tables
      driveMode.setString(currentDriveMode.toString());
    }
  }

  public void checkTip(){

    //Competition bot has coupled pitch/roll of 2.5 degrees during yaw
    //NOTE: PITCH IS FRONT/BACK OF ROBOT, Positive towards intake
    //NOTE: ROLL IS POSITIVE WITH CLOCKWISE ROTATION (LOOKING FROM BACK TOWARDS INTAKE)
    //Y direction is left/right, positive towards left when facing intake from back


    double kOffBalanceAngleThresholdDegrees = 4.0;
    double kOnBalanceAngleThresholdDegrees = 3.0;

    double pitchAngleDegrees = sensors.getPitch();    
    double rollAngleDegrees = sensors.getRoll();

    //enter tip correction mode if either roll or pitch is high enough
    if (!tip_correction_mode &&
        (Math.abs(pitchAngleDegrees)>kOnBalanceAngleThresholdDegrees ||
         Math.abs(rollAngleDegrees)>kOnBalanceAngleThresholdDegrees))
    {
      tip_correction_mode = true;
      //System.out.println("***TIP CORRECTION: Pitch="+pitchAngleDegrees);
    }
    
    //exit tip correction mode if both are low enough
    if (tip_correction_mode &&
              (Math.abs(pitchAngleDegrees)<kOffBalanceAngleThresholdDegrees &&
               Math.abs(rollAngleDegrees)<kOffBalanceAngleThresholdDegrees))
    {
      tip_correction_mode = false;
      //System.out.println("***END TIP CORRECTION***");

      //zero out the factors if leaving tip correction mode
      roll_factor = 0;
      pitch_factor = 0;

      currentCmd.setPitchCorrection(pitch_factor);
      currentCmd.setRollCorrection(roll_factor);
      
    }

    //feed PIDs, get correction factors
    if (tip_correction_mode){

      tipRollPid.setSetpoint(0); 
      roll_factor = tipRollPid.calculate(rollAngleDegrees);
      nt_roll_factor.setDouble(roll_factor);
  
      tipPitchPid.setSetpoint(0); 
      pitch_factor = tipPitchPid.calculate(-pitchAngleDegrees);
      nt_pitch_factor.setDouble(pitch_factor);

      //pass correction factors down to drive command
        currentCmd.setPitchCorrection(pitch_factor);
        currentCmd.setRollCorrection(roll_factor);
    }

    SmartDashboard.putNumber("Current Pitch", pitchAngleDegrees);
    SmartDashboard.putNumber("Current Roll", rollAngleDegrees);
    SmartDashboard.putBoolean("Tip Correction Mode", tip_correction_mode);
    SmartDashboard.putNumber("Pitch Correction Factor", pitch_factor);
    SmartDashboard.putNumber("Roll Correction Factor", roll_factor);

   }

  public void checkPid(){
       //For Pitch/Roll PID Tuning, comment out when done.
       requested_pitch_P = SmartDashboard.getNumber("Requested Pitch P", requested_pitch_P);
       requested_pitch_I = SmartDashboard.getNumber("Requested Pitch I", requested_pitch_I);
       requested_pitch_D = SmartDashboard.getNumber("Requested Pitch D", requested_pitch_D);
       requested_roll_P = SmartDashboard.getNumber("Requested Roll P", requested_roll_P);
       requested_roll_I = SmartDashboard.getNumber("Requested Roll I", requested_roll_I);
       requested_roll_D = SmartDashboard.getNumber("Requested Roll D", requested_roll_D);
       SmartDashboard.putNumber("Current Roll P", tipRollPid.getP());
   
       if (requested_pitch_P != tipPitchPid.getP()){
         tipPitchPid.setP(requested_pitch_P);
         System.out.println("****Tip Pitch P adjusted to: " + tipPitchPid.getP());
       }
       if (requested_pitch_I != tipPitchPid.getI()){
         tipPitchPid.setI(requested_pitch_I);
         System.out.println("****Tip Pitch I adjusted to: " + tipPitchPid.getI());
       }
       if (requested_pitch_D != tipPitchPid.getD()){
         tipPitchPid.setD(requested_pitch_D);
         System.out.println("****Tip Pitch D adjusted to: " + tipPitchPid.getD());
       }
   
       if (requested_roll_P != tipRollPid.getP()){
         tipRollPid.setP(requested_roll_P);
         System.out.println("****Tip Roll P adjusted to: " + tipRollPid.getP());
       }
       if (requested_roll_I != tipRollPid.getI()){
         tipRollPid.setI(requested_roll_I);
         System.out.println("****Tip Roll I adjusted to: " + tipRollPid.getI());
       }
       if (requested_roll_D != tipRollPid.getD()){
         tipRollPid.setD(requested_roll_D);
         System.out.println("****Tip Roll D adjusted to: " + tipRollPid.getD());
       }
  }

  // Waits delay mS for a shooting solution, or will return true after wait to
  // just shoot...
  public boolean shootWithoutTarget(double delay) {
    double now = RobotController.getFPGATime() / 1000.0;
    // added post season to shoot without LL solution
    // waits delay amount then returns true 
    return  (shoot_on_time > 0) && ((now - shoot_on_time) >= delay);
  }

}
