// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;
import frc.chadbot.RobotSpec_ChadBot;
import frc.lib2202.builder.Robot;
import frc.robot2024.RobotSpec_CompBot2024;
import frc.timbot.RobotSpec_TimBot;
import frc.robot2025.RobotSpec_AlphaBot2025;
import frc.robot2025.RobotSpec_BetaBot2025;
import frc.robot2025.RobotSpec_BotOnBoard;
import frc.robot2025.RobotSpec_BotOnBoard2;
import frc.robot2025.RobotSpec_BotOnBoard3;

public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    //new RobotSpecDefault();  //example only, don't load spec
    new RobotSpec_CompBot2024();
    new RobotSpec_ChadBot();
    new RobotSpec_TimBot();

    // 2025 sub-tree
    new RobotSpec_BetaBot2025();
    new RobotSpec_AlphaBot2025(); 
    new RobotSpec_BotOnBoard();
    new RobotSpec_BotOnBoard2();
    new RobotSpec_BotOnBoard3();

  }
  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}   
