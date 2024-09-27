// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.chadbot.RobotContainer;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;



public class ConstantBasicShootCommand extends Command{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    int ballCount = 999;

    NetworkTable table;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry ntBallRPS;
    NetworkTableEntry shooterState;
    
    ShooterSettings  cmdSS;         // instance the shooter sees
    ShooterSettings  prevSS;        // instance for prev State

    private boolean finished = false;

    final ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        Shooting("Shooting");

        String name;

        private Stage(String name){
            this.name = name;
        }

        public String toString(){
            return name;
        }
    }Stage stage;
    
    public ConstantBasicShootCommand(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        ntBallVel = table.getEntry("BallVel");
        ntBallRPS = table.getEntry("BallRPS");
        shooterState = table.getEntry("ShooterState");

        ntBallVel.setDouble(0);
        ntBallRPS.setDouble(0);

        cmdSS = new ShooterSettings(); //defaultShooterSettings SUS 
        prevSS = new ShooterSettings(cmdSS);

        stage = Stage.DoNothing;
        shooter.off();
    }

    @Override
    public void execute(){
        shooterState.setString(stage.toString());

        cmdSS = defaultShooterSettings;
        shooter.spinup(cmdSS);

        magazine.driveWheelOn(0.70);
    }

    @Override
    public void end(boolean interrupted){
        //magazine.driveWheelOff();
        //shooter.off();
    }

    public void setFinished(){
        finished = false;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }
}
