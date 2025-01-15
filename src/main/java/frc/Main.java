// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib2202.builder.Robot;
import frc.lib2202.builder.RobotSpecDefault;
import frc.robot2024.RobotSpec_AlphaBot2024;
import frc.robot2024.RobotSpec_ChadBot;
import frc.robot2024.RobotSpec_CompBot2024;
import frc.robot2024.RobotSpec_DoofBot;
import frc.robot2024.RobotSpec_SwerveBot;

public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    new RobotSpecDefault();
    new RobotSpec_AlphaBot2024();
    new RobotSpec_CompBot2024();
    new RobotSpec_ChadBot();
    new RobotSpec_SwerveBot();
    new RobotSpec_DoofBot();

  }
  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}
