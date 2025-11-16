package frc.robot2025;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.robot2025.Constants.CAN;
import frc.robot2025.commands.WristFLAToPos;
import frc.robot2025.subsystems.WristFLA;

public class RobotSpec_BotOnBoard3 implements IRobotSpec {

    // Robot Speed Limits
    RobotLimits robotLimits = null;
    
    // Chassis
    ChassisConfig chassisConfig = null;

    // SubsystemConfig gets registered in static array to match serial number at
    // Construct call
    SubsystemConfig subsystemConfig = new SubsystemConfig("bot-On-Board-3", "0326F275")
            // deferred construction via Supplier<Object> lambda
            .add(PowerDistribution.class, "PDP", () -> {
                var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
                pdp.clearStickyFaults();
                return pdp;
            })
            .add(HID_Subsystem.class, "DC", () -> {
                return new HID_Subsystem(0.3, 0.9, 0.05);
            })
            //.add(Climber.class);
            .add(WristFLA.class);

    public RobotSpec_BotOnBoard3() {
        // add the specs to the ssconfig
        subsystemConfig.setRobotSpec(this);
    }

    @Override
    public RobotLimits getRobotLimits() {
        return robotLimits;
    }

    @Override
    public IHeadingProvider getHeadingProvider() {
        return null; // no sensors in default, example for your bot's Spec:
        // return RobotContainer.getSubsystem(Sensors_Subsystem.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        return null;
    }

    @Override
    public void setBindings() {
        // FOR BOT ON BOARD you can configure bindings directly here
        // and avoid messing with BindingsOther or Comp.
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");

        if (dc.Operator() instanceof CommandXboxController) {
            //operator.povUp().whileTrue(new ClimberVelMove(0.75));
            //operator.povDown().whileTrue(new ClimberVelMove(-0.75));
            //operator.rightBumper().onTrue(new ClimberPosition(3.0, 0.75));
        } else if (dc.Operator() instanceof CommandPS4Controller operator) {
            operator.triangle().onTrue(new WristFLAToPos(WristFLA.PICKUP_POSITION,"triangle"));
            operator.square().onTrue(new WristFLAToPos(WristFLA.Q3_POSITION,"square"));
            operator.circle().onTrue(new WristFLAToPos(WristFLA.MID_POSITION,"circle"));
            operator.cross().onTrue(new WristFLAToPos(WristFLA.DROP_POSITION, "cross"));
        }
    }

    @Override
    public void setDefaultCommands() {

    }

}
