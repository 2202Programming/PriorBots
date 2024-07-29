package frc.lib2202.util;

import edu.wpi.first.math.geometry.Translation2d;

public class AprilTag2d {
    public Translation2d location;
    public int ID;

    public AprilTag2d(int id, Translation2d location) {
        this.ID = id;
        this.location = location;
    }

    public AprilTag2d(int id, double x, double y){
        this(id, new Translation2d(x,y));
    }

}
