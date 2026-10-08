package frc.robot2026.subsystems.Shooter;

public interface IFlyWheel {
    // all sets use flywheel edge linear speed.
    boolean atSetpoint();

    public IFlyWheel setSetpoint(double vel);
    
    double getSetpoint();

    IFlyWheel setVelocityTolerance(double vel_tolerance);

    double getTolerance();
    
    double getVelocity();

    // Motor Info
    double getOutputCurrent();
    double getMotorTemperature();
    double getAppliedOutput();

    // some methods that can help with testing calibration
    double getPosition();
    IFlyWheel setPosition(double pos);  

    double getPosRot();

    double getMotorRPM();


    void update_hardware();
}
