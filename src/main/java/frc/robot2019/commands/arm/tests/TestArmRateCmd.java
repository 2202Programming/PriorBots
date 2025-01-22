package frc.robot2019.commands.arm.tests;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.util.RateLimiter;
import frc.robot2019.commands.util.RateLimiter.InputModel;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.Constants;
import frc.robot2019.OI;
public class TestArmRateCmd extends ParallelCommandGroup {

    RateLimiter armRC;
    RateLimiter extenderRC;

    final ArmSubsystem arm;
    final OI m_oi;

    public TestArmRateCmd() {
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        m_oi = RobotContainer.getObject("OI");
        armRC = new RateLimiter(Constants.dT,
                this::getShoulderCmd, 
                arm::getRealAngle, 
                arm.PHI_MIN, // ShoulderMinDegrees,
                arm.PHI_MAX, // ShoulderMaxDegrees,
                -3.0, // dx_falling  
                5.0, // dx_raising
                InputModel.Position); // expo


        extenderRC = new RateLimiter(Constants.dT,
            this::getExtenderCmd, 
            arm::getExtension,
            5.0,  //Robot.arm.EXTEND_MIN, // inches,
            15.0, //Robot.arm.EXTEND_MAX, // inches
            -5.0, // dx_falling
            5.0,  // dx_raising
            InputModel.Position); 

        class RateCmd extends Command {
            final RateLimiter rc;
            final DoubleConsumer outfunct;

            RateCmd(RateLimiter _rc, DoubleConsumer _outfunct) {
                addRequirements(arm);
                rc = _rc;
                outfunct = _outfunct;
            }

            @Override
            public void initialize() { rc.initialize();   }

            @Override
            public void execute() { 
                rc.execute();
                outfunct.accept(rc.get());
            }

            @Override
            public boolean isFinished() { return false;  }
        }

        RateCmd shoulderCmd = new RateCmd(armRC, arm::setAngle);
        RateCmd extenderCmd = new RateCmd(extenderRC, arm::setExtension);
        addCommands(shoulderCmd, extenderCmd);
    }

    public void log() {
        SmartDashboard.putNumber("rc:sh:cmd", armRC.get() );
        SmartDashboard.putNumber("rc:ext:cmd", extenderRC.get()); 
    }
    // ### TODO: move the binding functions to a less hidden place - DPL
    // Bind the control to our functions
    public double getShoulderCmd() {
        return m_oi.getAssistantController().getRightY(); //Hand.kRight);
    }
    public double getExtenderCmd() {
        return m_oi.getAssistantController().getRightX(); //Hand.kRight);
    }
}