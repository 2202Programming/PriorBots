package frc.lib2202.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IHeadingProvider {
    public Rotation2d getRotation2d();

    default public Rotation2d getHeading() { return getRotation2d(); }
}
