// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;

public class Indexer extends SubsystemBase {

  final SparkFlex controller;
  final SparkFlexConfig controllerCfg;
  final RelativeEncoder encoder;
  final SparkClosedLoopController closedLoopController;
  final FeedForwardConfig ffObj;

  final DigitalInput indexGate; 

  final ClosedLoopSlot PositionSlot = ClosedLoopSlot.kSlot0;

  double posCF = 1.0 / 9.0; // [ROT]
  double velCF = 1.0 / (9.0 * 60.0); // [RPS] of the INDEXER, not the MOTOR

  double cruiseVel = 5767.0;
  double maxAccel = 10000.0;

  double P = 0.3;
  double I = 0.0;
  double D = 0.0;

  double iMaxAccum = 0.015;
  double iZone = 20.0;

  double kV = 1.12; // Volts / max RPM
  double kS = 0.0; // amount of power required to overcome any mechanical slop and to make it
                   // barely move
  double kA = 0.0; // constant of acceleration - seems to increase acceleration

  // Operational Variables
  double vel_setpoint;
  double pos_setpoint;
  double increment_position = 6.0; // rough estimate as of 2/20/2026
  boolean m_changes = false;
  boolean loaded;

  public Indexer(int CanID, boolean inverted, int dio_gate) {
    setName(inverted ? "indexer_left" : "indexer_right");
    indexGate = new DigitalInput(dio_gate);
    controller = new SparkFlex(CanID, MotorType.kBrushless);
    controllerCfg = new SparkFlexConfig();
    encoder = controller.getEncoder();
    closedLoopController = controller.getClosedLoopController();
    ffObj = controllerCfg.closedLoop.feedForward;
    configure(PositionSlot, inverted);
    configureTuning(PositionSlot);
    encoder.setPosition(0.0); // tells the motor it's at pos 0

    this.getWatcherCmd();  //todo remove for comp

    // Default command will keep indexer loaded but stops before flywheel
    this.setDefaultCommand(this.new Load()); 
  }

  private void configure(ClosedLoopSlot slot, boolean inverted) {

    controllerCfg
        .inverted(inverted)
        .idleMode(IdleMode.kBrake);

    controllerCfg.encoder
        .positionConversionFactor(posCF)
        .velocityConversionFactor(velCF);

    controllerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    controllerCfg.closedLoop.maxMotion
        .cruiseVelocity(cruiseVel, slot)
        .maxAcceleration(maxAccel, slot)
        .allowedProfileError(1);

    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void configureTuning(ClosedLoopSlot slot) {
    controllerCfg.closedLoop
        .p(P, slot).i(I, slot).d(D, slot) // PID
            .feedForward
        .kS(kS, slot).kV(kV, slot); // SV
  }

  public void setPosSetpoint(double setpoint) {
    closedLoopController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl, PositionSlot);
    pos_setpoint = setpoint;
  }

  public double getPosSetPoint() {
    return pos_setpoint;
  }

  public double getPositionError() {
    return Math.abs(encoder.getPosition() - pos_setpoint);
  }

  public double getAmps() {
    return controller.getOutputCurrent();
  }

  public void zeroPos() {
    encoder.setPosition(0.0);
  }

  public void setPct(double pct) {
    controller.set(pct);
  }
  
  public double getPct() {
    return controller.get();
  }
  
  public boolean hasFuel(){
    return !indexGate.get();
  }

  public Command cmdSetPct(double pct) {
    return runOnce(() -> {
      setPct(pct);
    });
  }

  void setkS(double newkS) {
    ffObj.kS(newkS);
    kS = newkS;
    m_changes = true;
  }

  void setkV(double newkV) {
    ffObj.kV(newkV);
    kV = newkV;
    m_changes = true;
  }

  public boolean isLoaded() {
    return loaded;
  }

  public void setLoaded(boolean value){
    loaded = value;
  }

  /**
   * 
   * @param motorController - The motor controller to apply the values to
   * @param motorConfig     - The cfg to send the values into
   * @param slot            - The slot (PositionSlot / VelocitySlot) to send the
   *                        values to
   * 
   *                        Updates values every frame for PID, kV, kS, iMaxAccum,
   *                        iZone, rampRate,
   */
  private void update(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
    // skip if no changes or no attached hw typical if use PIDF without calling
    if (!m_changes || motorConfig == null || motorController == null) return;

    motorConfig.closedLoop.pid(P, I, D, slot);
    motorConfig.closedLoop.feedForward.sva(kS, kV, kA, slot);

    // send to HW if we have a pid change, use async so robot loop isn't delayed
    motorController.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_changes = false;
  }

  @Override
  public void periodic() {
    update(controller, controllerCfg, PositionSlot);  //changes only when actively tuning via Elastic
  }

  public void setTestBindings(CommandXboxController xbox) {
    xbox.b().onTrue(new InstantCommand(() -> { zeroPos(); }));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("hasFuel", this::hasFuel, null);
    builder.addDoubleProperty("pos_cmd", this::getPosSetPoint, this::setPosSetpoint);
    builder.addDoubleProperty("pos_err", this::getPosSetPoint, null);

    builder.addDoubleProperty("amps", this::getAmps, null);

    builder.addDoubleProperty("kS", () -> {
      return kS;
    }, this::setkS);
    builder.addDoubleProperty("kV", () -> {
      return kV;
    }, this::setkV);

    builder.addDoubleProperty("P", () -> {
      return P;
    }, (double v) -> {
      this.P = v;
      m_changes = true;
    });

    builder.addDoubleProperty("I", () -> {
      return I;
    }, (double i) -> {
      this.I = i;
      m_changes = true;
    });

    builder.addDoubleProperty("D", () -> {
      return D;
    }, (double d) -> {
      this.D = d;
      m_changes = true;
    });
  }
  
  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
        return this.new IndexerWatcher();
  }

  // watcher will put values on the network tables for viewing in elastic
  class IndexerWatcher extends WatcherCmd {
    IndexerWatcher() {           
        addEntry("pct", Indexer.this::getPct, 2);
        addEntry("current", Indexer.this::getAmps, 2);       
     }
  }

  public class Load extends Command {
    final static double DEFAULT_SPEED = 1.0;  //pct power
    final static double BackupSpeed = -0.3;

    final double speed;    

    public Load() {
      this(DEFAULT_SPEED);
    }

    public Load(double _speed) {
      this.speed = _speed;
      this.addRequirements(Indexer.this);
    }

    @Override
    public void initialize() {
      loaded = true;
      if (!hasFuel()) {
        Indexer.this.setPct(speed);
        loaded = false;        // when done shooting, we should re-run init via reschedule
      }
    }

    @Override
    public void execute() {
      //stop on fuel
      if (Indexer.this.hasFuel() ) { // just broke gate, backup until hasFuel is false
        Indexer.this.setPct(BackupSpeed);
        loaded = true;
      }
      else if (!Indexer.this.hasFuel() && loaded ) {
        Indexer.this.setPct(0.0);   // done backing up
      }
      else if (!loaded) {
        Indexer.this.setPct(speed);
      }
    }

    @Override
    public void end(boolean interrupted) {
      Indexer.this.setPct(0.0);
    }

    @Override
    public boolean isFinished() {
      return false;    //used as default command, so never end...
    }

  }


}