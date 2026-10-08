package frc.robot2026.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Flywheel handles motor and gearing for the shooter flywheels.
 * 
 */
public class FlyWheelRev implements IFlyWheel {
  
  // Hardware
  final SparkMax controller; // this could be a generic controller controller...
  final SparkMaxConfig controllerCfg;
  final RelativeEncoder encoder;
  final SparkClosedLoopController closedLoopController;
  final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;
  final FlyWheelConfig cfg;
  final double posConverionFactor;

  // operational vars
  double vel_setpoint;
  double vel_tolerance = 0.5; // [m/s] flywheel edge linear velocity ~fuel speed

  public FlyWheelRev(int CAN_ID, FlyWheelConfig cfg) {
    this.cfg = cfg;
    controllerCfg = new SparkMaxConfig();
    controller = new SparkMax(CAN_ID, SparkMax.MotorType.kBrushless);
    posConverionFactor = 2.0 * Math.PI * cfg.flywheelRadius * cfg.gearRatio;
    // reset controller to factory
    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controllerCfg.inverted(cfg.inverted)
        .closedLoopRampRate(cfg.rampRate)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(cfg.stallAmp, cfg.freeAmp)
            // use physical used, m or m/s as seen at edge of flywheel, should be 1/2 fuel
            // cell velocity [m/s]
            .encoder // set driveEncoder to use units of the wheelDiameter, meters
        .positionConversionFactor(posConverionFactor) // [m]
        .velocityConversionFactor(posConverionFactor / 60.0); // [m/s]
    controllerCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // send values to hardware
    cfg.hw_pid.copyTo(controller, controllerCfg, kSlot);
    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = controller.getClosedLoopController();
    encoder = controller.getEncoder();

    setIMaxAccum(cfg.iMaxAccum);

    setSetpoint(0.0);
  }

  public void update_hardware() {
      //writes to hw, does nothing if there are no pid related changes
      cfg.hw_pid.copyChangesTo(controller, controllerCfg, kSlot);
    }

  // internal if we need to do something in subsys
  SparkBase getController() {
    return controller;
  }

  SparkClosedLoopController getClosedLoopController() {
    return closedLoopController;
  }

  // API used by sub-system
  public FlyWheelRev setSetpoint(double vel) { // setVelocitySetpoint
    vel_setpoint = vel;
    if (vel_setpoint == 0.0) {
      //force mode change to %pwr at zero, let it spin down, don't drive it to zero vel
      controller.set(vel_setpoint);
    }
    else {
      // closed loop velocity
      closedLoopController.setSetpoint(vel_setpoint, ControlType.kVelocity);
    }
    return this;
  }
  
  public double getSetpoint() {
    return vel_setpoint;
  }

  public double getVelocity() {
      return encoder.getVelocity();
    }

  public double getMotorRPM() {
      // return motor RPM by applying inverse velocity converstion factor
      return getVelocity() * 60.0 / posConverionFactor; 
    }

  public double getTolerance() {
    return vel_tolerance;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public FlyWheelRev setPosition(double pos) {
    encoder.setPosition(pos);
    return this;
  }


  public double getPosRot() {
    return encoder.getPosition() / posConverionFactor;
  }

  double getIAccum() {
    return closedLoopController.getIAccum();
  }

  public FlyWheelRev setVelocityTolerance(double vel_tolerance) {
      this.vel_tolerance = vel_tolerance;
      return this;
    }

  public boolean atSetpoint() {
      return Math.abs(getVelocity() - vel_setpoint) <= vel_tolerance;
    }

  //expose ramprate so we can test its effect and tune to soften power spikes
  double getRampRate() {
    return cfg.rampRate;
  }
  
  FlyWheelRev setRampRate(double rate) {
    cfg.rampRate = rate;
    controllerCfg.closedLoopRampRate(rate);
    controller.configureAsync(controllerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    return this;
  }

  //expose iMaxAccum for tuning
  double getIMaxAccum(){
    return cfg.iMaxAccum;
  }

  FlyWheelRev setIMaxAccum(double iMaxAccum) {
    cfg.iMaxAccum = iMaxAccum;
    controllerCfg.closedLoop.iMaxAccum(iMaxAccum, kSlot);
    controller.configureAsync(controllerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    return this;
  }

  @Override
  public double getOutputCurrent() {
    return controller.getOutputCurrent();
  }

  @Override
  public double getMotorTemperature() {
    return controller.getMotorTemperature();
  }

  @Override
  public double getAppliedOutput() {
    return controller.getAppliedOutput();
  }


}