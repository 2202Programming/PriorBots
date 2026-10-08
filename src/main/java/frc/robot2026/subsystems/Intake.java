// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

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
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;

public class Intake extends SubsystemBase {

  final SparkFlex controller;
  final SparkFlexConfig config;
  final RelativeEncoder encoder;
  final SparkClosedLoopController closedLoopController;
  final FeedForwardConfig ffObj;

  final double GearRatio = 3.0;
  final double conversionFactor = 1.0 / GearRatio; // [rot (mtr) / rot (output)]

  final ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

  final boolean motor_inverted = true;

  final DigitalInput lightgate;

  final double cruiseVel = 5767.0;
  final double maxAccel = 10000.0;

  double P = 0.7;
  double I = 0.0015;
  double D = 0.0;
  double kS = 0.0;
  double kV = 12.0 / 5767.0;

  double pos_setpoint;
  boolean m_changes = false;

  boolean disable_servo = true;

  // Servo speed/positions
  double cmdPos;
  double cmdPct;

  /** Creates a new Intake. */
  public Intake() {
    setName("Intake-" + CAN.IntakeID);
    lightgate = new DigitalInput(DigitalIO.IntakeGate);

    controller = new SparkFlex(CAN.IntakeID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    encoder = controller.getEncoder();
    closedLoopController = controller.getClosedLoopController();

    ffObj = config.closedLoop.feedForward;

    config
        .inverted(motor_inverted);

    config.encoder
        .positionConversionFactor(conversionFactor);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.closedLoop.maxMotion
        .cruiseVelocity(cruiseVel, slot)
        .maxAcceleration(maxAccel, slot)
        .allowedProfileError(1);

    config.closedLoop
        .p(P, slot).i(I, slot).d(D, slot)
        .feedForward
          .kS(kS, slot).kV(kV, slot);

    controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //don't use Trigger, changed to be a default command
    this.setDefaultCommand(cmdRunWhileFuel(.45, 0.5) );
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
    if (!m_changes || motorConfig == null || motorController == null)
      return;

    motorConfig.closedLoop.pid(P, I, D, slot);
    motorConfig.closedLoop.feedForward.sv(kS, kV, slot);

    // send to HW if we have a pid change, use async so robot loop isn't delayed
    motorController.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_changes = false;
  }

  @Override
  public void periodic() {
    update(controller, config, slot);
  }

  public void setPosSetpoint(double setpoint) {
    closedLoopController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl, slot);
    pos_setpoint = setpoint;
  }

  public Command cmdSetPos(double setpoint) {
    return runOnce(() -> {
      setPosSetpoint(setpoint);
    });
  }

  public Command cmdZeroPos() {
    return runOnce(() -> {
      zeroPos();
    });
  }

  public void setkS(double newkS) {
    ffObj.kS(newkS);
    kS = newkS;
    m_changes = true;
  }

  public void setkV(double newkV) {
    ffObj.kV(newkV);
    kV = newkV;
    m_changes = true;
  }

  public double getPosSetPoint() {
    return pos_setpoint;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public double getPositionError() {
    return Math.abs(getPosition() - pos_setpoint);
  }

  public boolean hasFuel() {
    return !lightgate.get();
  }

  public void zeroPos() {
    encoder.setPosition(0.0);
  }

  // velocity control only used for testing, normal cmds will use position
  public void setPercent(double pct) {
    cmdPct = pct;
    controller.set(pct);
  }

  public Command cmdPctPwr(double cmd_pct) {
    return Commands.runOnce(() -> { 
      this.setPercent(cmd_pct);
    }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // this holds the command until released
  public Command cmdEject(double cmd_pct) {
    return Commands.run(() -> { 
      this.setPercent(cmd_pct);
    });
  }

  public void setTestBindings(CommandXboxController opr) {
    opr.leftBumper()
        .whileTrue(this.cmdPctPwr(0.5))
        .onFalse(this.cmdPctPwr(0.0));

    opr.rightBumper()
        .onTrue(this.cmdPctPwr(0.75))
        .onFalse(this.cmdPctPwr(0.0));

    opr.b()
        .onTrue(this.cmdPctPwr(0.0));

    opr.b().onTrue(new InstantCommand(() -> {
      zeroPos();
    }));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("hasFuel", this::hasFuel, null);
    builder.addDoubleProperty("pos_cmd", this::getPosSetPoint, this::setPosSetpoint);
    builder.addDoubleProperty("pos_err", this::getPosSetPoint, null);
    builder.addDoubleProperty("pos", this::getPosition, null);
    builder.addDoubleProperty("RPM", this::getRPM, null);

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

  public Command cmdRunWhileFuel(double pct, double seconds) {
    return this.new RunWhileFuel(pct, seconds);
  }

  class RunWhileFuel extends Command {
    double pct;
    double seconds;
    Timer no_fuel_timer;

    public RunWhileFuel(double pct, double seconds) {
      this.pct = pct;
      this.seconds = seconds;
      no_fuel_timer = new Timer();
      this.addRequirements(Intake.this);
    }

    @Override
    public void initialize() {
      no_fuel_timer.restart();
      Intake.this.setPercent(pct);
      //System.out.println("Intake::RunWhileFuel init");
    }

    @Override
    public void execute() {
      // keep resetting timer as long as we see fuel
      if (Intake.this.hasFuel() ) { 
        no_fuel_timer.restart();
        Intake.this.setPercent(pct);
      }

      //Stop running after no fuel seen for elapsed seconds
      if (no_fuel_timer.hasElapsed(seconds))
         Intake.this.setPercent(0.0);
    }

    public void end(boolean interrupted) {
      Intake.this.setPercent(0.0);
      //System.out.println("Intake::RunWhileFuel end");
    }

    // this is a default command, it never finishes, but may be canceled
    public boolean isFinished(){
     return false;
    }

  }


}