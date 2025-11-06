package frc.robot2025;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.robot2025.Constants.CAN;
import frc.robot2025.commands.GroundIntake.BtmArmGoToPos;
import frc.robot2025.commands.GroundIntake.BtmArmVel;
import frc.robot2025.commands.GroundIntake.PickupSequence;
import frc.robot2025.commands.GroundIntake.SetZero;
import frc.robot2025.commands.GroundIntake.TopArmVel;
import frc.robot2025.commands.GroundIntake.TopHold;
import frc.robot2025.subsystems.CycloidalDrive;
import frc.robot2025.subsystems.GroundIntake;

//copy or extend this code for your robot - remember to override:
// TBD
//
public class RobotSpec_BotOnBoard2 implements IRobotSpec {

    // Build

    // Robot Speed Limits
    RobotLimits robotLimits = new RobotLimits(
            FeetPerSecond.of(15.0),
            DegreesPerSecond.of(360.0));

    // Chassis
    double kWheelCorrectionFactor = .995;
    double kSteeringGR = 12.8;
    double kDriveGR = 8.14;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    ChassisConfig chassisConfig = new ChassisConfig(
            MperFT * (21.516 / 12.0) / 2.0, // X offset [m]
            MperFT * (24.87 / 12) / 2.0, // Y offset [m]
            kWheelCorrectionFactor, // []
            kWheelDiameter, // [m]
            kSteeringGR, // []
            kDriveGR); // []

    // SubsystemConfig gets registered in static array to match serial number at
    // Construct call
    SubsystemConfig subsystemConfig = new SubsystemConfig("bot-On-Board-2", "03061025")
            // deferred construction via Supplier<Object> lambda
            .add(PowerDistribution.class, "PDP", () -> {
                var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
                pdp.clearStickyFaults();
                return pdp;
            })
            .add(HID_Subsystem.class, "DC", () -> {
                return new HID_Subsystem(0.3, 0.9, 0.05);
            })            
            .add(CycloidalDrive.class)
    ;

    public RobotSpec_BotOnBoard2() {
        // finish BetaBot's drivePIDF
        chassisConfig.drivePIDF.setIZone(0.2);
        // add the specs to the ssconfig
        subsystemConfig.setRobotSpec(this);
    }
   
    @Override
    public void setBindings() {
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");
        CommandXboxController operator = (CommandXboxController) dc.Operator();
        // keep for testing purposes -er

        operator.rightBumper().whileTrue(new BtmArmVel(30.0));
        operator.leftBumper().whileTrue(new BtmArmVel(-30.0));
        operator.povRight().whileTrue(new TopArmVel(30.0));
        operator.povLeft().whileTrue(new TopArmVel(-30.0));
        operator.a().onTrue(new BtmArmGoToPos(0.0));
        
        operator.b().onTrue(new SetZero());
        operator.rightTrigger().whileTrue(new TopHold(5.0));
        // testing real commands -er
        operator.x().whileTrue(new PickupSequence("c"));
        operator.y().whileTrue(new PickupSequence("a"));

    }

}
