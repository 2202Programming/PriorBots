// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.subsystems;

import static frc.chadbot.Constants.CAN;
import static frc.chadbot.Constants.PCM1;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.chadbot.Constants.Intake;

public class Intake_Subsystem extends SubsystemBase {
    /**
     * Intake arm consists of a deploy/retractable arm, controlled by Double Solenoid
     * Intake itself consists of a wheel, controlled by Spark PWM value
     */
    // defaults - move to constants
    final double IntakeMotorStrength = 0.6;
    final double SideMotorStrength = 0.5;

    //Localized Constants - what valve value does what action
    static final Value DEPLOY  = Value.kReverse;
    static final Value RETRACT = Value.kForward;

    //slot to use on controllers
    int slot = 0;

    //Instantiations
    final CANSparkMax intake_mtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(CAN.PCM1,
                PneumaticsModuleType.CTREPCM,
                PCM1.INTAKE_UP_SOLENOID_PCM,
                PCM1.INTAKE_DOWN_SOLENOID_PCM);
    
    private CANSparkMax r_side_mtr = new CANSparkMax(CAN.MAG_R_SIDE_MTR, MotorType.kBrushless);
    private CANSparkMax l_side_mtr = new CANSparkMax(CAN.MAG_L_SIDE_MTR, MotorType.kBrushless);

    //Constructor
    public Intake_Subsystem(){
        Intake.r_side_mtrPIDF.copyTo(r_side_mtr.getPIDController(), slot);
        Intake.l_side_mtrPIDF.copyTo(l_side_mtr.getPIDController(), slot);
        r_side_mtr.clearFaults();
        r_side_mtr.restoreFactoryDefaults();
        r_side_mtr.setInverted(true);
    
        l_side_mtr.clearFaults();
        l_side_mtr.restoreFactoryDefaults();
        l_side_mtr.setInverted(false);
    }

    //Set the Intake Mode

    //Turn Intake Motor On by sending a double value
    public void on(double intakeMotorStrength, double sideMotorStrength) {
        intake_mtr.set(intakeMotorStrength);
        sidesOn(sideMotorStrength);
    }

    //turn on horizontal intake only
    public void horizontalOn(double intakeMotorStrength){
        intake_mtr.set(intakeMotorStrength);
    }

    /**
     * Loads cargo
     */
    public void defaultOn(){
        on(IntakeMotorStrength, SideMotorStrength);
    }       

     // used by gated magazine control
    public void sidesOn(double sideMotorStrength) {
        r_side_mtr.set(sideMotorStrength);
        l_side_mtr.set(sideMotorStrength);
    }

    public void sidesOff() {
        sidesOn(0.0);
    }

    //Turn Intake Motor Off by setting a double value
    public void off() {
        intake_mtr.set(0.0);
        sidesOff();
    }

    //Deploy arm mechanism using a Double Solenoids
    public void deploy() {
        intake_solenoid.set(DEPLOY);
    }

    //Retract arm mechanism using a Double Solenoids
    public void retract() {
        intake_solenoid.set(RETRACT);
    }
    
    //Returns the state of the Intake Arm
    public boolean isDeployed() {
      return ( intake_solenoid.get() == DEPLOY); 
    }
}

