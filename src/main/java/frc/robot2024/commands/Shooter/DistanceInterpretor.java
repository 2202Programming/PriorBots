// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.Constants.Tag_Pose;
import frc.lib2202.command.TargetWatcherCmd;
import frc.lib2202.subsystem.OdometryInterface;

/** Add your docs here. */
public class DistanceInterpretor extends TargetWatcherCmd {

    static DistanceInterpretor singleton = null;

    // only need one of these, use singleton pattern
    public static DistanceInterpretor getSingleton() {
        if (singleton == null) {
            singleton = new DistanceInterpretor();
        }
        return singleton;
    }

    // hook into drivetrain for distance    
    final OdometryInterface odo;
    Translation2d targetTranslation2d; // speaker target location

    InterpolatingTreeMap<Double, Double> ang_table;
    InterpolatingTreeMap<Double, Double> rpm_table;
    double meas_dist; // set in calc

    private DistanceInterpretor() {        
        odo = RobotContainer.getSubsystemOrNull("odometry");

        InverseInterpolator<Double> distance = InverseInterpolator.forDouble();
        Interpolator<Double> angle = Interpolator.forDouble();
        Interpolator<Double> rpm = Interpolator.forDouble();

        ang_table = new InterpolatingTreeMap<>(distance, angle);
        rpm_table = new InterpolatingTreeMap<>(distance, rpm);

        // set up table with measured values
        // RPM, limelight distance d4 tag, calculating angle
        ang_table.put(1.900, 45.0); //3000
        ang_table.put(2.140, 44.0);
        ang_table.put(2.499, 42.0); //3000
        ang_table.put(3.092, 38.0); //3000
        //tested up to here 3/25 nr/ko
        ang_table.put(3.5855, 36.0); //3000
        ang_table.put(3.5856, 34.2); //3500
        ang_table.put(3.842, 32.5);
        ang_table.put(4.030, 32.3); 
        ang_table.put(4.250, 32.0);
        ang_table.put(4.399, 31.4);
        ang_table.put(4.500, 29.0); // 4000
        ang_table.put(4.930, 30.0); // 4000
        ang_table.put(5.440, 28.6); // 4500
        ang_table.put(5.610, 29.0); // 4500
        ang_table.put(5.611, 45.0);
        ang_table.put(20.0, 45.0);

        rpm_table.put(1.900, 3000.0);
        rpm_table.put(2.140, 3000.0);
        rpm_table.put(2.499, 3000.0);
        rpm_table.put(3.092, 3000.0);
        rpm_table.put(3.549, 3000.0);
        rpm_table.put(3.5855, 3000.0);
        rpm_table.put(3.5856, 3500.0);
        rpm_table.put(3.842, 3500.0);
        rpm_table.put(4.030, 3500.0);
        rpm_table.put(4.250, 3500.0);
        rpm_table.put(4.399, 3500.0);
        rpm_table.put(4.4400, 4000.0);
        rpm_table.put(4.500, 4000.0);
        rpm_table.put(4.930, 4000.0);
        rpm_table.put(4.9301, 4500.0);
        rpm_table.put(5.440, 4500.0);
        rpm_table.put(5.610, 4500.0);
        rpm_table.put(5.611, 4000.0);
        rpm_table.put(20.0, 4000.0);
        
        // ensure we have a default speaker target, based on alliance
        setTarget();  
    }

    // return interpolated angle from a distance
    public double getTargetAngleFromDistance(double distance) {
        return ang_table.get(distance);
    }

    public double getRPMFromDistance(double distance) {
        return rpm_table.get(distance);
    }

    // Implementations for TargetWatcherCmd
    public void calculate() {
        meas_dist = odo.getDistanceToTranslation(targetTranslation2d);
    }

    public double getTargetAngle() {
        return getTargetAngleFromDistance(meas_dist);
    }

    public double getTargetRPM() {
        return getRPMFromDistance(meas_dist);
    }

    public double getTargetDistance() {
        return meas_dist;
    }

    // set targetTrlansation2d based on alliance
    public void setTarget() {
        // deal with Optional<> , can be null when simulating without driverstation. 
        var optAlliance = DriverStation.getAlliance();
        var alliance = optAlliance.isPresent() ? optAlliance.get() : DriverStation.Alliance.Blue;
        targetTranslation2d = (alliance == DriverStation.Alliance.Blue) ? 
                Tag_Pose.ID7.location : // Blue Alliance
                Tag_Pose.ID4.location; // Red Alliance

        if (!optAlliance.isPresent())
            System.out.println("Warning: Defaulting to Blue Alliance in ContinousAngleTracker cmd.");
        // call calculate() to ensure update if target changed
        calculate();
    }

    public void setTarget(Translation2d newTarget){
        targetTranslation2d = newTarget;
        // call calculate() to ensure update if target changed
        calculate();
    }

    @Override
    public boolean necessaryForCompetition() {return true;}

}
