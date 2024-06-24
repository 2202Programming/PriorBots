package frc.lib2202.builder;

public class Config {
    // This config does nothing, no subsystems added but will let any new RoboRio Build
    public static final SubsystemConfig botOnBoardDefaultSN = 
    new SubsystemConfig("DEFAULT:bot-On-Board", System.getenv("serialnum"));
  
}
