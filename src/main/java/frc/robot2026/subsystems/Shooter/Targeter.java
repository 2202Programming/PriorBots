package frc.robot2026.subsystems.Shooter;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.TargeterInterface;
import frc.lib2202.subsystem.UX.TrimTables.Trim;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.Constants.TheField;

/*
manage shooter speeds for different command use
    manual speed 
    ranged speed  
    distance to hub
    heading to  hub 
 */
public class Targeter extends SubsystemBase implements TargeterInterface {

    // VelocityEntry uses a persistent Trim value and will update vel_table entry on
    // changes
    class VelocityEntry {
        final double distance; // used as key into interpMap
        final Trim speedTrim;

        VelocityEntry(String name, double distance_ft, double speed) {
            this.distance = distance_ft * MperFT;
            speedTrim = new Trim("Targeter", name, speed);
            double tbl_speed = speedTrim.getValue();
            Targeter.this.vel_table.put(distance, tbl_speed);
            // use callback to update vel_map on change
            speedTrim.addChangeCallback(this::callback);
        }

        // return isn't used, just a useful supplier
        Boolean callback() {
            double newSpeed = speedTrim.getValue();
            Targeter.this.vel_table.put(distance, newSpeed); // update table with new value
            return true;
        }
    }

    final double HIGH_SPEED = 29.5; // [M/S]
    final double LOW_SPEED = 20.8; // [M/S]
    final double LOW_TOLERANCE = 0.5; // [M/S]
    final double UNBLOCK_SPEED = -15.0; // [M/S]

    final double MAX_SHOOTING_DISTANCE = 10.0; //[M]
    final double MAX_MOTION_ADJUSTMENT = 5.0; //[M]
    final double SHOOTING_DISTANCE_MOTION_ADJUSTMENT_FACTOR = 0.1; //[magic number units]
    final double SHOOTING_MOTION_ADJUSTMENT_FACTOR = 0.2; //[magic number units]

    // todo make this a trim entry
    final double dist_err = MperFT * 0.0 / 12.0; // [m] testing tape measure seemed like we were 6" short

    // Provided by vince as angle between the center of the motor and the trailing
    // edge of the ball exit ramp
    final double Shooter_Angle = 65.0; // [deg]

    final OdometryInterface odo;
    final DriveTrainInterface dt;

    // Hub targets
    public final Translation2d blueHubTarget;
    public final Translation2d redHubTarget;

    final InverseInterpolator<Double> distance = InverseInterpolator.forDouble();
    final Interpolator<Double> vel_mps = Interpolator.forDouble();
    final Interpolator<Double> tolerance_mps = Interpolator.forDouble();
    final Interpolator<Double> hangTime = Interpolator.forDouble();
    final InterpolatingTreeMap<Double, Double> vel_table = new InterpolatingTreeMap<>(distance, vel_mps); // [m][m/s]
    final InterpolatingTreeMap<Double, Double> tolerance_table = new InterpolatingTreeMap<>(distance, tolerance_mps); // [m][m/s]
    final InterpolatingTreeMap<Double, Double> hangTime_table = new InterpolatingTreeMap<>(distance, hangTime); // [m][s]

    Translation2d targetTranslation2d;
    double target_dist; // function of VPE pose and Hub center + math
    double target_dist_motion_corrected;
    double target_speed = LOW_SPEED;
    double target_speed_motion_corrected = LOW_SPEED;
    double manual_speed = LOW_SPEED; // flywheel speed manually controlled by driver
    double target_tolerance = LOW_TOLERANCE;
    double override_dist = 0.0; // non-zero will skip LL distance calcs
    double prevDistance;
    double currentHangTime;
    double parallelTargetVelocity;

    Translation2d motionTargetTranslation2d;
    double motionDeltaX;
    double motionDeltaY;
    double motionDeltaSpeed;

    final Field2d m_field;
    final FieldObject2d m_field_obj;

    public Targeter() {
        this("vision_odo", "drivetrain");
    }

    public Targeter(String odo_name) {
        this(odo_name, "drivetrain");
    }

    public Targeter(String odo_name, String drivetrain_name) {
        setName("Targeter");
        // use hub tags to calc center of blue and red hubs
        Pose3d blue1 = TheField.fieldLayout.getTagPose(20).get();
        Pose3d blue2 = TheField.fieldLayout.getTagPose(26).get();
        blueHubTarget = new Translation2d((blue1.getX() + blue2.getX()) / 2.0, blue1.getY());

        Pose3d red1 = TheField.fieldLayout.getTagPose(4).get();
        Pose3d red2 = TheField.fieldLayout.getTagPose(10).get();
        redHubTarget = new Translation2d((red1.getX() + red2.getX()) / 2.0, red1.getY());

        new TargeterWatcher();

        targetTranslation2d = blueHubTarget;

        odo = RobotContainer.getSubsystem(odo_name);
        dt = RobotContainer.getSubsystem(drivetrain_name);
        // Quick and dirty table measured on 2/21/26
        // distance[m] -> flywheel [m/s]
        // Create Velocity table with VelocityEntry so it is tied to persistent trims
        new VelocityEntry("00.0 ft", 0.0, 14.3);
        new VelocityEntry("05.4 ft", 5.4, 14.3);
        new VelocityEntry("06.0 ft", 6.0, 15.3);
        new VelocityEntry("07.0 ft", 7.0, 15.7);
        new VelocityEntry("08.0 ft", 8.0, 16.1);
        new VelocityEntry("09.0 ft", 9.0, 16.9);
        new VelocityEntry("10.0 ft", 10.0, 17.65);
        new VelocityEntry("11.0 ft", 11.0, 18.6);
        new VelocityEntry("12.3 ft", 12.3, 19.6);
        new VelocityEntry("14.0 ft", 14.0, 20.55);
        new VelocityEntry("17.0 ft", 17.0, 24.2);
        new VelocityEntry("20.0 ft", 20.0, 24.3);

        /****************
         * old way
         * vel_table.put(0.0 * MperFT, 18.5); // set a min
         * vel_table.put(5.4 * MperFT, 18.5); //1:1
         * vel_table.put(6.0 * MperFT, 19.2); //1:1
         * vel_table.put(10.0 * MperFT, 22.9); // 1:1 this is ladder radius
         * vel_table.put(12.3 * MperFT, 25.9); // 1:1
         * vel_table.put(14.0 * MperFT, 28.1); // 1:1
         * vel_table.put(17.0 * MperFT, 31.0); // 1:1
         * //vel_table.put(25.0 * MperFT, 31.0); // set a max
         *******************/

        tolerance_table.put(0.0 * MperFT, 1.0); // 1.6);
        tolerance_table.put(5.0 * MperFT, 1.0); // 1.4);
        tolerance_table.put(6.0 * MperFT, 1.0); // 1.2 );
        tolerance_table.put(10.0 * MperFT, 1.0); // 0.8);
        tolerance_table.put(17.0 * MperFT, 1.0);

        // made up values, please measure
        // hangtime is seconds (input in meters)
        hangTime_table.put(0.0 * MperFT, 1.0);
        hangTime_table.put(5.0 * MperFT, 1.5);
        hangTime_table.put(10.0 * MperFT, 2.0);
        hangTime_table.put(15.0 * MperFT, 2.5);

        m_field = new Field2d();
        m_field_obj = m_field.getObject("VirtualTarget");
        SmartDashboard.putData("FieldTargeter", m_field);
    }

    @Override
    public void periodic() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        targetTranslation2d = (alliance == Alliance.Blue) ? blueHubTarget : redHubTarget;

        target_dist = (override_dist == 0.0) ? odo.getDistanceToTranslation(targetTranslation2d) : override_dist;
        target_dist += dist_err;
        target_speed = vel_table.get(target_dist);
        target_tolerance = tolerance_table.get(target_dist);

        // first guess at target based on stationary distance
        prevDistance = motionCorrectedDistance(targetTranslation2d);
        motionTargetTranslation2d = motionCorrectedTarget(hangTime_table.get(prevDistance));

        // converge on motiondistance
        for (int i = 0; i < 1; i++) {
            // new distance to last motiontarget
            double distance = odo.getDistanceToTranslation(motionTargetTranslation2d);
            motionTargetTranslation2d = motionCorrectedTarget(hangTime_table.get(distance));
            if (Math.abs((distance - prevDistance)) < 0.1)
                break;
            prevDistance = distance;
        }

        target_dist_motion_corrected = (override_dist == 0.0) ? odo.getDistanceToTranslation(motionTargetTranslation2d)
                : override_dist;
        target_dist_motion_corrected += dist_err;
        target_speed_motion_corrected = vel_table.get(target_dist_motion_corrected);

        motionDeltaX = targetTranslation2d.getX() - motionTargetTranslation2d.getX();
        motionDeltaY = targetTranslation2d.getY() - motionTargetTranslation2d.getY();
        motionDeltaSpeed = target_speed_motion_corrected - target_speed;
        currentHangTime = hangTime_table.get(prevDistance);
        m_field.setRobotPose(odo.getPose());
        m_field_obj.setPose(new Pose2d(motionTargetTranslation2d, new Rotation2d()));

    }

    // Expose hub locations for commands
    public Translation2d getRedHub() {
        return redHubTarget;
    }

    public Translation2d getBlueHub() {
        return blueHubTarget;
    }

    public void setManualSpeed(double value) {
        manual_speed = value;
    }

    public double getManualSpeed() {
        return manual_speed;
    }

    public double getTolerance() {
        return target_tolerance;
    }

    public double getManualTolerance() {
        return 0.5;
    }

    public double getTargetSpeed() {
        return target_speed;
    }

    // Call this on autoInit and teleInit to make sure alliance target is set
    public void setTarget() {
        var optAlliance = DriverStation.getAlliance(); // make sure this is accurate :)
        var alliance = optAlliance.isPresent() ? optAlliance.get() : Alliance.Blue;
        targetTranslation2d = (alliance == Alliance.Blue ? blueHubTarget : redHubTarget);

        if (!optAlliance.isPresent())
            System.out.println(
                    "Warning: As no alliance was recived, we are defaulting to the Blue Alliance hub. This will go badly");
    }

    // Command interface for changing manual speed
    public Command manualHigh() {
        return runOnce(() -> {
            manual_speed = HIGH_SPEED;
        });
    }

    public Command manualLow() {
        return runOnce(() -> {
            manual_speed = LOW_SPEED;
        });
    }

    /*
     * use this cmd to manually override the LL target speed, if 0.0 uses Vision's
     * Distance.
     */
    public Command OverrideTargetDistanceFT(double distance_ft) {
        return runOnce(() -> {
            override_dist = distance_ft * MperFT;
        });
    }

    // new point on the field to be the aiming target based on our velocity vector
    // and ball hang time
    public Translation2d motionCorrectedTarget(double hangTime) {
        double xVelocity = dt.getFieldRelativeSpeeds().vxMetersPerSecond;
        double yVelocity = dt.getFieldRelativeSpeeds().vyMetersPerSecond;
        double xAdjustment = xVelocity * hangTime * SHOOTING_MOTION_ADJUSTMENT_FACTOR; //[M]
        double yAdjustment = yVelocity * hangTime * SHOOTING_MOTION_ADJUSTMENT_FACTOR; //[M]

        //don't adjust target beyond ability to shoot
        if (xAdjustment > MAX_MOTION_ADJUSTMENT) xAdjustment = MAX_MOTION_ADJUSTMENT;
        if (yAdjustment > MAX_MOTION_ADJUSTMENT) yAdjustment = MAX_MOTION_ADJUSTMENT;
        if (xAdjustment < -MAX_MOTION_ADJUSTMENT) xAdjustment = -MAX_MOTION_ADJUSTMENT;
        if (yAdjustment < -MAX_MOTION_ADJUSTMENT) yAdjustment = -MAX_MOTION_ADJUSTMENT;

        return new Translation2d(targetTranslation2d.getX() - xAdjustment,
                targetTranslation2d.getY() - yAdjustment);
    }

    // get a new target distance based on velocity vector directly towards/away from
    // target
    public double motionCorrectedDistance(Translation2d t) {

        // 1. Vector towards target (meters)
        double dx = t.getX() - odo.getPose().getX();
        double dy = t.getY() - odo.getPose().getY();

        // 2. Distance (Magnitude) [m]
        double distance = Math.sqrt(dx * dx + dy * dy);

        // 3. Unit vector (normalized direction)
        double ux = dx / distance;
        double uy = dy / distance;

        // current velocities, field coordinates [m/s]
        double vx = dt.getFieldRelativeSpeeds().vxMetersPerSecond;
        double vy = dt.getFieldRelativeSpeeds().vyMetersPerSecond;

        // 4. Dot product (projection of velocity onto direction) [m/s]
        parallelTargetVelocity = vx * ux + vy * uy;
        double distanceAdjustment = hangTime_table.get(distance) * parallelTargetVelocity * SHOOTING_DISTANCE_MOTION_ADJUSTMENT_FACTOR; // [m]
        double finalMotionCorrectedDistance = distanceAdjustment + distance;

        //don't return a distance further than we can shoot
        if (finalMotionCorrectedDistance > MAX_SHOOTING_DISTANCE) finalMotionCorrectedDistance = MAX_SHOOTING_DISTANCE;
        if (finalMotionCorrectedDistance < -MAX_SHOOTING_DISTANCE) finalMotionCorrectedDistance = -MAX_SHOOTING_DISTANCE;

        return finalMotionCorrectedDistance;
    }

    public double getMotionTargetX() {
        return motionTargetTranslation2d.getX();
    }

    public double getMotionTargetY() {
        return motionTargetTranslation2d.getY();
    }

    @Override
    public Translation2d getMotionCorrectedTarget() {
        return motionTargetTranslation2d;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // if no setters, prefer watcher
        builder.addDoubleProperty("manual_speed_sb", this::getManualSpeed, this::setManualSpeed);

        // builder.addDoubleProperty("target_speed", () -> {
        // return this.target_speed;
        // }, null);
        // builder.addDoubleProperty("manual_speed", () -> {
        // return this.manual_speed;
        // }, null);
    }

    class TargeterWatcher extends WatcherCmd {
        TargeterWatcher() {
            addEntry("isLowSpeed", () -> {
                return Targeter.this.getManualSpeed() == Targeter.this.LOW_SPEED;
            }, 2);
            addEntry("manual_speed", Targeter.this::getManualSpeed, 2);
            addEntry("target_dist-ft", () -> {
                return Targeter.this.target_dist / MperFT;
            }, 2);
            addEntry("target_dist-m", () -> {
                return Targeter.this.target_dist;
            }, 2);
            addEntry("target_speed", () -> {
                return Targeter.this.target_speed;
            }, 2);
            addEntry("MotionDeltaX", () -> {
                return Targeter.this.motionDeltaX;
            }, 2);
            addEntry("MotionDeltaY", () -> {
                return Targeter.this.motionDeltaY;
            }, 2);
            addEntry("MotionDeltaSpeed", () -> {
                return Targeter.this.motionDeltaSpeed;
            }, 2);
            addEntry("preDistance", () -> {
                return Targeter.this.prevDistance;
            }, 2);
            addEntry("hangTime", () -> {
                return Targeter.this.currentHangTime;
            }, 2);
            addEntry("ParallelVel", () -> {
                return Targeter.this.parallelTargetVelocity;
            }, 2);
        }
    }
}