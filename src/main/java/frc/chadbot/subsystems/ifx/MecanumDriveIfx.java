package frc.chadbot.subsystems.ifx;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MecanumDriveIfx extends Subsystem {
    void drive_normalized (double x, double y, double rotation);
}
