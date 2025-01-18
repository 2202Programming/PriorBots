package frc.robot2019;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot2019.commands.arm.TeleopArmControlCommand;
import frc.robot2019.commands.arm.tests.TestArmRateCmd;
import frc.robot2019.commands.intake.tests.TestWristPositionCommand;


public class RobotTest {
    private XboxController driver = RobotSpec_2019.m_oi.getDriverController();
    private XboxController assistant = RobotSpec_2019.m_oi.getAssistantController();
    private XboxController switchBoard = RobotSpec_2019.m_oi.getAssistantController();

    // TESTING Started in TestInit
    Command testWristCmd;
    TestArmRateCmd testArmCmd;
    private Command armTest;

    public RobotTest() {
    }

    public void initialize() {
        // Set commands here so they override the OI 
        CommandScheduler.getInstance().removeAll();
        // remove defaultCommands so only testing is being done.
        RobotSpec_2019.intake.setDefaultCommand(null);
        RobotSpec_2019.gearShifter.setDefaultCommand(null);
        RobotSpec_2019.arm.zeroArm();

        // TESTING Commands, only get scheduled if we enter Test mode
        //testWristCmd = new  TestWristPositionCommand(this::Wrist_AssistLeftTrigger);
        armTest = new TeleopArmControlCommand(this::leftJoyY, this::rightJoyY);
        
        //armTest.start();
        //testWristCmd.start();
    }

    public void periodic() {
        logSmartDashboardSensors();
    }

    /**
     * 
     * Bind the Joystick control functions here and use DoubleSupplier function arguemnts to pass
     * them into your test functions. 
     * 
     * This keeps all the Joystick bindings out of the bowels of the code and signals can be 
     * modified as needed.  
     * 
     * 
     */
    private double Wrist_AssistLeftTrigger() {
        //rescale as expected by wrist test
        double temp = -1.0 +2.0*RobotSpec_2019.m_oi.getAssistantController().getTriggerAxis(Hand.kLeft);
        return temp;
    }

    private double leftJoyY() {
        return Math.abs(assistant.getY(Hand.kLeft)) < 0.05? 0: -assistant.getY(Hand.kLeft);
    }

    private double rightJoyY() {
        return Math.abs(assistant.getY(Hand.kRight)) < 0.05? 0: -assistant.getY(Hand.kRight); 
    }
       
    private void logSmartDashboardSensors() {
        // SmartDashboard.putNumber("Left Encoder Count", driveTrain.getLeftEncoderTalon().getSelectedSensorPosition());
        // SmartDashboard.putNumber("Left Encoder Rate", driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Right Encoder Count", driveTrain.getRightEncoderTalon().getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Encoder Rate", driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
        // SmartDashboard.putString("Gear Shifter State", String.valueOf(gearShifter.getCurGear()));

        RobotSpec_2019.arm.log();
        RobotSpec_2019.arm.logTalons();
 
        SmartDashboard.putData(Scheduler.getInstance()); 
        //SmartDashboard.putData(driveTrain);
        //SmartDashboard.putData(gearShifter);
      }
    
      private void resetAllDashBoardSensors() {
        RobotSpec_2019.driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
        RobotSpec_2019.driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
      }
}