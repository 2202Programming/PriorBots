package frc.robot2019.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot2019.RobotMap;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Authors: Derek Laufenberg
 *          Billy Huang
 *          Alexander Ge
 *         
 * Changes:  
 * 2/10/2019   DPL  Created first draft of climber subsystem
 *                  
 * 
 */

/**
 * The Climber is no longer on the robot, code kept as example. 1/17/2025
 * 
 * Climber subsystem consistes of an extensible foot to lift and lower the robot
 * with. When the robot is lifted up, a motor on the foot is used to roll the
 * robot past the step so the robot can be lowered on the higher step.
 *
 * A gear/pawl mechanism driven by a solenoid prevents the foot from backsliding
 * when rasing or lowering.
 *
 */

public class ClimberSubsystem extends ExtendedSubSystem {
    private long logTimer = System.currentTimeMillis();

    // constants
    // Pawl solenoid Extend/Retract - confirmed 3/23/2019
    public final DoubleSolenoid.Value Extend = Value.kReverse; 
    public final DoubleSolenoid.Value Retract = Value.kForward;
    
    //DrawerSlide 
    public final DoubleSolenoid.Value HoldSlide = Value.kReverse;
    public final DoubleSolenoid.Value ReleaseSlide = Value.kForward;

    //use for pawl burp sequence
    public final double STALL_POWER_EXTEND = 0.6;  //Power needed to allow pawl to fire while extended up 
    public final double STALL_POWER_RETRACT = -0.4; //Power needed to allow pawl to fire while on the ground

    public final double COUNTS_PER_IN = 13.3;      //updated with Kevin's meas 3/23/2019
    public final double IN_PER_COUNT = 1.0 / COUNTS_PER_IN;

    // physical devices
    DoubleSolenoid pawl = new DoubleSolenoid(RobotMap.CLIMB_PCM_ID, RobotMap.moduleType, 
        RobotMap.CLIMB_PAWL_ENGAGE_PCM, RobotMap.CLIMB_PAWL_RELEASE_PCM);
    DoubleSolenoid drawerSlide = new DoubleSolenoid(RobotMap.CLIMB_PCM_ID, RobotMap.moduleType, 
        RobotMap.CLIMB_SLIDE_PULL_PCM, RobotMap.CLIMB_SLIDE_RELEASE_PCM);

    SparkMax footExtender = new SparkMax(RobotMap.CLIMB_FOOT_SPARK_MAX_CAN_ID, MotorType.kBrushless);
    SparkMax roller = new SparkMax(RobotMap.CLIMB_ROLLER_SPARK_MAX_CAN_ID, MotorType.kBrushed);
    SparkMaxConfig footCfg = new SparkMaxConfig();
    SparkMaxConfig rollerCfg = new SparkMaxConfig();
    
    private DigitalInput extensionAtMax = new DigitalInput(RobotMap.CLIMB_MAX_EXTENSION_CH);
    private DigitalInput extensionAtMin = new DigitalInput(RobotMap.CLIMB_MIN_EXTENSION_CH);
    private DigitalInput drawerSlideAtMin = new DigitalInput(RobotMap.CLIMB_DRAWER_SLIDE_MIN);

    // think we need to add an encoder
    public ClimberSubsystem() {
        super("Climber");
        footCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        footCfg.absoluteEncoder.positionConversionFactor(COUNTS_PER_IN)
            .velocityConversionFactor(COUNTS_PER_IN / 60.0)
            .inverted(false);
        footExtender.configure(footCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        rollerCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rollerCfg.absoluteEncoder.positionConversionFactor(1.0)  //rpms
            .velocityConversionFactor(1.0 / 60.0)
            .inverted(false);
        roller.configure(rollerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        zeroClimber();
    }

    public Command zeroSubsystem() {
        return new edu.wpi.first.wpilibj2.command.InstantCommand(() ->{
            //do nothing, don't have a climber anymore
        });   //ClimbZero();
    }

    public void zeroClimber() {
        footExtender.getEncoder().setPosition(0.0);
        roller.getEncoder().setPosition(0.0);
    }

    public void setDrawerSlide(boolean on) {
        if (on)
            drawerSlide.set(Extend);
        else
            drawerSlide.set(Retract);
    }

    public void setDrawerSlide(DoubleSolenoid.Value cmd) 
    {
        drawerSlide.set(cmd);
    }

    public void setPawl( DoubleSolenoid.Value cmd) {
        pawl.set(cmd);
    }
    
    //use in test code - but T/F is not good for physics
    public void setPawl(boolean on) {
        if (on)
            pawl.set(HoldSlide);
        else
            pawl.set(ReleaseSlide);
    }

    public void setExtenderSpeed(double speed) {
        footExtender.set(speed);
    }

    public double getExtenderSpeed() {
        return footExtender.get();
    }

    public double getExtension() {
        return footExtender.getEncoder().getPosition();
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public double getRollerSpeed() {
        return roller.get();
    }

    public boolean footAtExtend() {
        //active low signal 
        return !extensionAtMax.get();
    }

    public boolean footAtRetract() {
        //active low
        return !extensionAtMin.get();
    }

    public boolean climberAgainstWall() {
        // Active low
        return !drawerSlideAtMin.get();
    }


    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ClimberSubsystem");
        builder.addDoubleProperty("ExtenderSpeed", this::getExtenderSpeed, this::setExtenderSpeed);
        builder.addDoubleProperty("FootSpeed", this::getRollerSpeed, this::setRollerSpeed);
    }

    public void log(int interval) {
        if ((logTimer + interval) < System.currentTimeMillis()) { 
            logTimer = System.currentTimeMillis();
            SmartDashboard.putNumber("Cl:ext", getExtension());
            SmartDashboard.putBoolean("Cl:foot_ext", footAtExtend());
            SmartDashboard.putBoolean("Cl:foot_ret", footAtRetract());
         }
    }
}