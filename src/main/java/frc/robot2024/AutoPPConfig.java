package frc.robot2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class AutoPPConfig {
    public static RobotConfig GUIconfig;

    //generic configuration - these settings have worked well in past
    public static void ConfigureAutoBuilder() {
        SwerveDrivetrain sdt = RobotContainer.getSubsystemOrNull(SwerveDrivetrain.class);
        var translationPID = new PIDConstants(7.0, 0.0, 0.0);
        var rotationPID = new PIDConstants(7.0, 0.0, 0.0);
        if (sdt != null) {
            ConfigureAutoBuilder(sdt, translationPID, rotationPID);
        }
    }

    // custom pids for SwerveDrivetrain
    public static void ConfigureAutoBuilder(PIDConstants translationPID, PIDConstants rotationPID) {
        SwerveDrivetrain sdt = RobotContainer.getSubsystemOrNull(SwerveDrivetrain.class);
        if (sdt != null) {
            ConfigureAutoBuilder(sdt, translationPID, rotationPID);
        }
    }

    // AutoBuilder for PathPlanner - uses internal static vars in AutoBuilder
    public static void ConfigureAutoBuilder(SwerveDrivetrain sdt,
            PIDConstants translationPID, PIDConstants rotationPID) {
        try {
            GUIconfig = RobotConfig.fromGUISettings();

            // Configure the AutoBuilder last
            AutoBuilder.configure(
                    sdt::getPose, // Robot pose supplier
                    sdt::autoPoseSet, // Method to reset odometry (will be called if your auto has a starting pose)
                    sdt::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    sdt::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    // HolonomicPathFollowerConfig
                    new PPHolonomicDriveController(translationPID, rotationPID),
                    GUIconfig,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // sdt will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    sdt); // Reference to this subsystem to set requirements

        } catch (Exception e) {
            // Handle exception as needed
            System.out.println("PATHING - Could not initialize PathPlanner\n" +
                    "PATHING - check for ~/deploy/pathplanner/settings.json");
            e.printStackTrace();
            System.out.println("PATHING - End of stack trace --------------");
        }
    }
}
