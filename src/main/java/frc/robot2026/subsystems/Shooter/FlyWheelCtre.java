package frc.robot2026.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlyWheelCtre implements IFlyWheel {
    final TalonFX m_fx;
    final VelocityVoltage m_velocityVoltage;
    final NeutralOut m_brake;

    final TalonFXConfiguration configs;
    final FlyWheelConfig cfg;
    final double converionFactor;

    double vel_setpoint_rps;    // [rps]
    double vel_tolerance_rps = 0.1; // [rps]

    public FlyWheelCtre(int CAN_ID, FlyWheelConfig _cfg) {
        CANBus canbus = new CANBus("rio");
        this.cfg = _cfg;
        m_fx = new TalonFX(CAN_ID, canbus);
        m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
        m_brake = new NeutralOut();
        configs = new TalonFXConfiguration();

        // ctre uses rot or rot-per-sec
        converionFactor = 2.0 * Math.PI * cfg.flywheelRadius * cfg.gearRatio; // [m/mtr-rot]

        // configure the talonFX with our cfg
        /*
         * Voltage-based velocity requires a velocity feed forward to account for the
         * back-emf of the motor
         */
        configs.Slot0
                .withKS(0.0) // To account for friction, add 0.1 V of static feedforward
                .withKV(cfg.hw_pid.getF()) 
                .withKP(cfg.hw_pid.getP())
                .withKI(cfg.hw_pid.getI())
                .withKD(cfg.hw_pid.getD());

        // CTRE claims magic that iZone and iAccumMax are not needed. I call BS.

        InvertedValue inverted = cfg.inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        configs.MotorOutput
                .withInverted(inverted)
                .withNeutralMode(NeutralModeValue.Coast);

        // Peak output of 12 volts - full range?
        configs.Voltage
                .withPeakForwardVoltage(Volts.of(12))
                .withPeakReverseVoltage(Volts.of(-12));

        configs.CurrentLimits
                .withStatorCurrentLimit(cfg.stallAmp)
                .withStatorCurrentLimitEnable(true);

        // arb-ff??
        // m_velocityVoltage.FeedForward = 0.0;
        // configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; // similar to
        // rampRate on rev?

        // This is a hack, but it is what ctre's demos suggest. I am starting to dislike
        // CTRE.
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_fx.getConfigurator().apply(configs, 1.0); // power up we can wait
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        //
        setPosition(0.0);
        setSetpoint(0.0);
    }

    @Override
    public IFlyWheel setSetpoint(double vel) {
        vel_setpoint_rps = vel / converionFactor;
        if (vel == 0.0) {
            m_fx.setControl(m_brake);
        } else {
            m_fx.setControl(m_velocityVoltage.withVelocity(vel_setpoint_rps));
        }
        return this;
    }

    @Override
    public double getSetpoint() {
        return vel_setpoint_rps * converionFactor;
    }

    @Override
    public double getVelocity() {
        return m_fx.getVelocity().getValueAsDouble() * converionFactor;
    }

    @Override
    public double getMotorRPM() {
        return m_fx.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public double getTolerance() {
        return vel_tolerance_rps * converionFactor;
    }

    @Override
    public IFlyWheel setVelocityTolerance(double vel_tolerance) {
        this.vel_tolerance_rps = vel_tolerance / converionFactor;
        return this;
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(m_fx.getVelocity().getValueAsDouble() - vel_setpoint_rps) <= vel_tolerance_rps;
    }

    @Override
    public double getPosition() {
        return m_fx.getPosition().getValueAsDouble() * converionFactor;
    }

    @Override
    public IFlyWheel setPosition(double pos) {
        m_fx.setPosition(pos / converionFactor); // convert to rot
        return this;
    }

    @Override
    public double getPosRot() {
        return m_fx.getPosition().getValueAsDouble();
    }

    @Override
    public double getOutputCurrent() {
        return m_fx.getStatorCurrent(false).getValueAsDouble();
    }

    @Override
    public double getMotorTemperature() {
        return m_fx.getDeviceTemp(false).getValueAsDouble();
    }

    @Override
    public double getAppliedOutput() {
        return m_fx.getBridgeOutput(false).getValueAsDouble();
    }

    @Override
    public void update_hardware() {
        // check for pid changes
        if (cfg.hw_pid.hasChanged()) {
            configs.Slot0
                .withKV(cfg.hw_pid.getF())
                .withKP(cfg.hw_pid.getP())
                .withKI(cfg.hw_pid.getI())
                .withKD(cfg.hw_pid.getD());

            //Send to hardware - this could be slow, I don't see an async like in rev
            StatusCode status = m_fx.getConfigurator().apply(configs);
            if (!status.isOK()) {
                //make some noise on failure
                System.out.println("CtreFlywheel live update failed, code: " + status.toString());
            }

            // mark changes sent to hw
            cfg.hw_pid.clearChanged();
        }

    }
}
