package frc.chadbot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chadbot.Constants.CAN;
import frc.chadbot.Constants.Intake;
import frc.chadbot.Constants.PCM1;

public class Intake_Subsystem extends SubsystemBase {
    /**
     * Intake arm consists of a deploy/retractable arm, controlled by Double
     * Solenoid
     * Intake itself consists of a wheel, controlled by Spark PWM value
     */
    // defaults - move to constants
    final double IntakeMotorStrength = 0.6;
    final double SideMotorStrength = 0.5;

    // Localized Constants - what valve value does what action
    static final Value DEPLOY = Value.kReverse;
    static final Value RETRACT = Value.kForward;

    // slot to use on controllers
    ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

    //Encoder Definitions
    final RelativeEncoder r_mtr_Encoder;
    final RelativeEncoder l_mtr_Encoder;
    final RelativeEncoder intake_Encoder;

    // Instantiations
    final SparkMax intake_mtr = new SparkMax(CAN.INTAKE_MTR, SparkMax.MotorType.kBrushless);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(CAN.PCM1,
            PneumaticsModuleType.CTREPCM,
            PCM1.INTAKE_UP_SOLENOID_PCM,
            PCM1.INTAKE_DOWN_SOLENOID_PCM);

    private SparkMax r_side_mtr = new SparkMax(CAN.MAG_R_SIDE_MTR, MotorType.kBrushless);
    private SparkMax l_side_mtr = new SparkMax(CAN.MAG_L_SIDE_MTR, MotorType.kBrushless);
    private SparkMaxConfig intake_mtr_cfg = new SparkMaxConfig();
    private SparkMaxConfig l_side_config = new SparkMaxConfig();
    private SparkMaxConfig r_side_config = new SparkMaxConfig();

    // Constructor
    public Intake_Subsystem() {
        l_side_config
            .inverted(true) //magic value
            .idleMode(IdleMode.kCoast);
        
        r_side_config
            .inverted(false) //magic value
            .idleMode(IdleMode.kCoast);

        intake_mtr_cfg
            .inverted(false) //magic value
            .idleMode(IdleMode.kCoast);
        Intake.r_side_mtrPIDF.copyTo(r_side_mtr, r_side_config, slot);
        Intake.l_side_mtrPIDF.copyTo(l_side_mtr, l_side_config, slot);

    
         r_side_mtr.configure(r_side_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        l_side_mtr.configure(l_side_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake_mtr.configure(intake_mtr_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        l_side_mtr.clearFaults();
        r_side_mtr.clearFaults();
        intake_mtr.clearFaults();
        //TODO add conversion factor??

        //l_side_mtr.configure(l_side_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       // r_side_mtr.configure(r_side_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //intakeMtrPid = l_side_mtr.getClosedLoopController();
        r_mtr_Encoder = r_side_mtr.getEncoder();
        l_mtr_Encoder = l_side_mtr.getEncoder();
        intake_Encoder = intake_mtr.getEncoder();


        // r_side_mtr.restoreFactoryDefaults();
        // r_side_mtr.setInverted(false); // nren 12-12-2022 electrical flipped this motor so changed from true to false

        // l_side_mtr.restoreFactoryDefaults();
        // l_side_mtr.setInverted(true); // nren 12-19-2022 electrical flipped this motor so changed from false to true
    }

    // Set the Intake Mode

    // Turn Intake Motor On by sending a double value
    public void on(double intakeMotorStrength, double sideMotorStrength) {
        intake_mtr.set(intakeMotorStrength);
        sidesOn(sideMotorStrength);
    }

    // turn on horizontal intake only
    public void horizontalOn(double intakeMotorStrength) {
        intake_mtr.set(intakeMotorStrength);
    }

    /**
     * Loads cargo
     */
    public void defaultOn() {
        on(IntakeMotorStrength, SideMotorStrength);
    }

    // used by gated magazine control
    public void sidesOn(double sideMotorStrength) {
        r_side_mtr.set(sideMotorStrength);
        l_side_mtr.set(sideMotorStrength);
    }

    public void sidesOff() {
        sidesOn(0.0);
    }

    // Turn Intake Motor Off by setting a double value
    public void off() {
        intake_mtr.set(0.0);
        sidesOff();
    }

    // Deploy arm mechanism using a Double Solenoids
    public void deploy() {
        intake_solenoid.set(DEPLOY);
    }

    // Retract arm mechanism using a Double Solenoids
    public void retract() {
        intake_solenoid.set(RETRACT);
    }

    // Returns the state of the Intake Arm
    public boolean isDeployed() {
        return (intake_solenoid.get() == DEPLOY);
    }
}
