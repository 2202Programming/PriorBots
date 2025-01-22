package frc.robot2019.commands.intake.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.Constants;
import frc.robot2019.OI;
import frc.robot2019.commands.util.RateLimiter;
import frc.robot2019.commands.util.RateLimiter.InputModel;
import frc.robot2019.subsystems.IntakeSubsystem;

public class TestWristRateCommand extends Command {
    RateLimiter wristRC;
    final IntakeSubsystem intake;
    final OI m_oi;

    public TestWristRateCommand() {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        m_oi = RobotContainer.getObject("OI");
        addRequirements(intake);
        wristRC = new RateLimiter(Constants.dT,
                this::getCmd, 
                intake::getAngle, 
                intake.WristMinDegrees -10, 
                intake.WristMaxDegrees+ 10, 
                -80.0, // dx_fall deg/sec 
                180.0,  // dx_raise deg/ses
                InputModel.Rate);
        
        //finish up scaling for rate
        wristRC.setRateGain(200.0);   // stick (-1, 1) *k = deg/sec comm  -/+200 deg/sec
        wristRC.setDeadZone(5.0);    // ignore rates less than 5.0 deg/sec
    }

    // Must supply a function to get a user's command in normalized units
    public double getCmd() {
        double   temp =  m_oi.getAssistantController().getLeftY(); //Hand.kLeft);
        return temp;
    }
    
    public void initialize() {
        wristRC.initialize();
    }

    
    public void execute() {
        wristRC.execute();
        intake.setAngle(wristRC.get());
    }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    
    public boolean isFinished() {
        return false;
    }

    public void log() {
        SmartDashboard.putNumber("TWR:RCout", wristRC.get());
    }
}