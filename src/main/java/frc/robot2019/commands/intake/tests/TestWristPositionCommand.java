package frc.robot2019.commands.intake.tests;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.util.RateLimiter;
import frc.robot2019.commands.util.RateLimiter.InputModel;
import frc.robot2019.subsystems.IntakeSubsystem;
import frc.robot2019.Constants;

public class TestWristPositionCommand extends Command {
    RateLimiter wristPC;
    final IntakeSubsystem intake;

    public TestWristPositionCommand(DoubleSupplier getter) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake);
        wristPC = new RateLimiter(Constants.dT,
                getter, 
                intake::getAngle,
                intake.WristMinDegrees, 
                intake.WristMaxDegrees, 
               -60.0,   // dx_fall deg/sec 
                180.0,  // dx_raise deg/se
                InputModel.Position);
        
        //finish up scaling for rate
        wristPC.setRateGain(100.0);   // trigger (-1, 1) *k = deg command
        wristPC.setDeadZone(10.0);    // ignore position less than 10.0 deg
    }

    
    public void initialize() {
        wristPC.initialize();
    }

    public void execute() {
        wristPC.execute();
        intake.setAngle(wristPC.get());
    }

    // This is just a test, it doesn't finish. Enjoy moving the wrist with the controller.
    public boolean isFinished() {
        return false;
    }

    public void log() {
        SmartDashboard.putNumber("TWP:RPout", wristPC.get());
    }
}