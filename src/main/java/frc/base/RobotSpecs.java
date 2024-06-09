// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.base;

import edu.wpi.first.wpilibj.RobotBase;
import frc.base.Configs;
import frc.base.Constants.DriveTrain;
import frc.robot2023.subsystems.Swerve.Config.CANConfig;
import frc.robot2023.subsystems.Swerve.Config.ChassisConfig;
import frc.robot2023.subsystems.Swerve.Config.ChassisInversionSpecs;
import frc.robot2023.subsystems.Swerve.Config.WheelOffsets;

public class RobotSpecs {
    public enum RobotNames {
        CompetitionBotAlpha2024("CompetitionBotAlpha2024"), 
        CompetitionBotBeta2024("CompetitionBotBeta2024"),
        SwerveBot("SwerveBot"),
        CompetitionBot2023("DoofBot2023"),
        ChadBot("ChadBot"),
        UnknownBot("UnknownBot"),
        BotOnBoard("BotOnBoard");

        String name;

        private RobotNames(String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }
    
    /*Configs*/
    public RobotNames myRobotName;
    private WheelOffsets myWheelOffsets;
    private ChassisConfig myChassisConfig;
    public SubsystemConfig mySubsystemConfig;
    private ChassisInversionSpecs myChassisInversionSpecs;
    private CANConfig myCANConfig;

    public RobotSpecs() {
        this(System.getenv("serialnum"));
    }

    public RobotSpecs(String serialNo) {
        myRobotName = getRobotName(serialNo);

        // if we are simulated, use the competionBot so we have everything
        if (RobotBase.isSimulation()) {
            myRobotName = RobotNames.CompetitionBotAlpha2024;
        }
        // setup to handle any swerve both we have
        switch (myRobotName) {
            case SwerveBot:
                myWheelOffsets = DriveTrain.swerveBotOffsets;
                myChassisConfig = DriveTrain.swerveBotChassisConfig;                
                myChassisInversionSpecs = DriveTrain.swerveBotChassisInversionSpecs;
                myCANConfig = DriveTrain.swerveBotCANConfig;
                mySubsystemConfig = Configs.swerveBotSubsystemConfig;
                break;
                
            case CompetitionBotAlpha2024:
                myWheelOffsets = DriveTrain.comp2024AlphaBotOffsets;
                myChassisConfig = DriveTrain.comp2024AlphaBotChassisConfig;                
                myChassisInversionSpecs = DriveTrain.comp2024BotAlphaInversionSpecs;
                myCANConfig = DriveTrain.comp2024BotCANConfig;
                mySubsystemConfig = Configs.comp2024AlphaBotSubsystemConfig;
                break;

            case CompetitionBotBeta2024:
                myWheelOffsets = DriveTrain.comp2024BetaBotOffsets;
                myChassisConfig = DriveTrain.comp2024BotBetaChassisConfig;                
                myChassisInversionSpecs = DriveTrain.comp2024BotBetaInversionSpecs;
                myCANConfig = DriveTrain.comp2024BotCANConfig;
                mySubsystemConfig = Configs.comp2024BetaBotSubsystemConfig;
                break;

            case CompetitionBot2023:  //doofBot
                myWheelOffsets = DriveTrain.doofBotOffsets;
                myChassisConfig = DriveTrain.doofBotChassisConfig;                
                myChassisInversionSpecs = DriveTrain.doofBotChassisInversionSpecs;
                myCANConfig = DriveTrain.doofBotCANConfig;
                mySubsystemConfig = Configs.doofBotSubsystemConfig;
                break;

            case ChadBot:
                myWheelOffsets = DriveTrain.chadBotOffsets;
                myChassisConfig = DriveTrain.chadBotChassisConfig;
                myChassisInversionSpecs = DriveTrain.chadBotChassisInversionSpecs;
                myCANConfig = DriveTrain.chadBotCANConfig;
                mySubsystemConfig = Configs.chadBotSubsystemConfig;
                break;

            default:
            case UnknownBot:
            case BotOnBoard:
                myWheelOffsets = DriveTrain.swerveBotOffsets;
                myChassisConfig = DriveTrain.swerveBotChassisConfig;                
                myChassisInversionSpecs = DriveTrain.swerveBotChassisInversionSpecs;
                myCANConfig = DriveTrain.swerveBotCANConfig;
                mySubsystemConfig = Configs.botOnBoardSystemConfig;
                System.out.println("***Non-driving robot,don't expect too much***");
                break;
        }
        System.out.println("***I am " + myRobotName + " ***");        
    }

    public String getRobotNameString() {
        return this.myRobotName.name;
    }

    public WheelOffsets getWheelOffset() {
        return myWheelOffsets;
    }

    public ChassisConfig getChassisConfig() {
        return myChassisConfig;
    }

    public SubsystemConfig getSubsystemConfig() {
        return mySubsystemConfig;
    }

    public ChassisInversionSpecs getChassisInversionSpecs(){
        return myChassisInversionSpecs;
    }

    public CANConfig getCANConfig(){
        return myCANConfig;
    }

    // takes the roborio serial # and returns the robot name
    public RobotNames getRobotName(String serialNo) {
        RobotNames tempRobotName;

        if (serialNo == null)
            return RobotNames.UnknownBot;

        if (serialNo.compareTo("031b7511") == 0)
            tempRobotName = RobotNames.SwerveBot;
        else if (serialNo.compareTo("03238151") == 0)
            tempRobotName = RobotNames.ChadBot;
        else if (serialNo.compareTo("0312db1a") == 0)
            tempRobotName = RobotNames.BotOnBoard;
        else if (serialNo.compareTo("031b7523") == 0)
            tempRobotName = RobotNames.CompetitionBot2023;
        else if (serialNo.compareTo("032381BF") == 0) 
            tempRobotName = RobotNames.CompetitionBotAlpha2024;
        else if (serialNo.compareTo("032D2062") == 0)
            tempRobotName = RobotNames.CompetitionBotBeta2024;
        else
            tempRobotName = RobotNames.UnknownBot;

        System.out.println("***RoboRio SERIAL NUM: " + serialNo);
        System.out.println("***Robot identified as: " + tempRobotName);
        return tempRobotName;
    }
}
