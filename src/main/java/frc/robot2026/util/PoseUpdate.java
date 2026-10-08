package frc.robot2026.util;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseUpdate{
    public Pose2d pose;
    public double timestamp;

    public PoseUpdate(Pose2d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
