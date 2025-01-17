package frc.lib2202.subsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.util.VisionWatchdog;

// Swerve Drive Train (SDT) must be created before Swerve-PoseEstimator

public class VisionPoseEstimator extends SubsystemBase {

    // set true if we found everything needed, otherwise this system is disabled
    final boolean correct_config;

    // This connects us to whatever gyro is being used for robot heading, configured
    // in RobotSpecs
    final IHeadingProvider sensors;
    final SwerveDrivetrain sdt; // must be lib2022 version
    final SwerveDriveOdometry m_odometry; // read-only here, updated in sdt
    final SwerveDriveKinematics kinematics; // const matrix based on chassis geometry, get from SDT
    SwerveModulePosition[] meas_pos; // provided by sdt

    Pose2d m_pose;  //based on odometry
    Pose2d old_pose;

    final VisionWatchdog watchdog;
    final double kTimeoffset; // typical ~= .1; // [s] measurement delay from photonvis
    final Limelight limelight;

    // Bearing calcs (TBD)
    // private double currentBearing = 0;
    //private double filteredBearing = 0;
    //private double filteredVelocity = 0;

    // Creates a new Single-Pole IIR filter
    // Filter Time constant is 0.1 seconds
    // Period is 0.02 seconds - this is the standard FRC main loop period
    //private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);
    //private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);

    boolean visionPoseUsingRotation = true; // read from sdt.useVisionRotation()
    boolean visionPoseEnabled = true;

    SwerveDrivePoseEstimator m_poseEstimator_ll;
    // monitor diffs in ll and odometry poses
    private double x_diff; // [m]
    private double y_diff; // [m]
    private double yaw_diff; // [deg]

    //vision systems limelight and photonvision(TBD)
    private Pose2d llPose;
    private Pose2d prev_llPose;
    private Pose2d pvPose = null; // TBD

    // field estimate based on vision estimate llPose
    public final Field2d m_field;

    // no-args ctor, default timings
    public VisionPoseEstimator() {
        this(0.1, 3.0); // typical settings
    }

    public VisionPoseEstimator(double kTimeoffset, double watchdog_interval) {
        this.kTimeoffset = kTimeoffset;
        watchdog = new VisionWatchdog(watchdog_interval);
        m_field = new Field2d();

        // other subsystems
        sdt = RobotContainer.getSubsystemOrNull(SwerveDrivetrain.class);
        sensors = RobotContainer.getRobotSpecs().getHeadingProvider();
        limelight = RobotContainer.getSubsystemOrNull(Limelight.class);

        // confirm config is correct
        correct_config = sdt != null && sensors != null && (limelight != null);

        if (sdt != null) {
            kinematics = sdt.getKinematics();
            meas_pos = sdt.getSwerveModulePositions();
            m_odometry = sdt.getOdometry();
            m_pose = sdt.getPose();
        } else {
            // no sdt, set the sdt related final vars
            kinematics = null;
            m_odometry = null;
            m_pose = new Pose2d();
            meas_pos = new SwerveModulePosition[] {
                    new SwerveModulePosition(), new SwerveModulePosition(),
                    new SwerveModulePosition(), new SwerveModulePosition()
            };
        }
        // Estimators
        initializeEstimator();
        // start the network monitor
        new VisionPoseEstimatorMonitorCmd();
    } // ctor

    @Override
    public void periodic() {
        if (!correct_config)
            return;

        m_pose = m_odometry.getPoseMeters();
        updateEstimator(); //sets llPose
        
        // apply llPose to robot position subject any constraints like velocity or distance 
        useEstimate();
        m_field.setRobotPose(llPose);

        // compare llPose and odometry pose 
       
        x_diff = Math.abs(llPose.getX() - m_pose.getX());
        y_diff = Math.abs(llPose.getY() - m_pose.getY());
        yaw_diff = Math.abs(llPose.getRotation().getDegrees() - m_pose.getRotation().getDegrees());
    }

    // helper functions
    void initializeEstimator() {
        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
         * The numbers used below are robot specific, and should be tuned.
         * 
         * TODO - add PID config to RobotSpecs
         */
        m_poseEstimator_ll = new SwerveDrivePoseEstimator(
                kinematics,
                sensors.getRotation2d(),
                meas_pos,
                m_pose, // was new Pose2d(), // initial pose () - dpl 1/2/2025 
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // std x,y, heading from odmetry
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // std x, y heading from vision

        //set initial values to odometry based m_pose
        llPose = prev_llPose = m_pose;
        // photon vision- TBD
        /*
         * m_poseEstimator_pv = new SwerveDrivePoseEstimator(
         * kinematics,
         * sensors.getRotation2d(),
         * meas_pos,
         * m_pose, // was new Pose2d(), // initial pose () - dpl 1/2/2025
         * VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // std x,y, heading
         * from odmetry
         * VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // std x, y heading
         * from vision
         */
    }

    /** Updates the field relative position of the robot. */
    void updateEstimator() {
        // Limelight update
        // this should happen every robot cycle, regardless of vision targets.
        m_poseEstimator_ll.update(sensors.getRotation2d(), meas_pos); // gyro and sdt position
        
        // TODO - check on timestamp references, focus on initialization t=0 maybe
        // global clock should be used

         // dpl modules[0] was used as proxy for robot speed, replace with field speed^2.
         ChassisSpeeds field_speed = sdt.getFieldRelativeSpeeds();
         double v2 = field_speed.vxMetersPerSecond * field_speed.vxMetersPerSecond +
                 field_speed.vyMetersPerSecond * field_speed.vyMetersPerSecond;
 
        // only apply ll measurements if we have tags, quality, and not moving too fast (2.5m/s)
        if (limelight.getNumApriltags() > 0 && //make sure at least 1 tag is in view
            //limelight.getTA() > 0.13 && //JR I don't think we want this here, the limelight is also filtering this out on it's end
            Math.abs(v2) < (2.5 * 2.5)) { //don't use vision if moving too fast - might be better in limelight 3G
            // this should happen only if we have a tag in view
            // OK if it is run only intermittanly. Uses latency of vision pose.
            m_poseEstimator_ll.addVisionMeasurement(limelight.getBluePose(), limelight.getVisionTimestamp());
        }
        // dpl - moved out of tag check, should be able to get estimate each frame even
        // without a tag based update.
        prev_llPose = llPose;
        llPose = m_poseEstimator_ll.getEstimatedPosition();
    }

    //set sdt's pose if it's enabled
    void useEstimate() {
        visionPoseUsingRotation = sdt.useVisionRotation();
        visionPoseEnabled = sdt.useVisionPose();
        
        Rotation2d current_rotation = m_pose.getRotation();
        if (visionPoseEnabled) {
            //TODO - dpl 1/2/2025 is watchdog needed???
            watchdog.update(llPose, prev_llPose);
            if (visionPoseUsingRotation) {
                // update robot pose, include vision-based rotation
                sdt.setPose(llPose);
            } else {
                // update robot translation, do not update rotation
                sdt.setPose(new Pose2d(llPose.getTranslation(), current_rotation));
            }         
        }
    }

    // Public API

    // force to visionEstimator's pose to given one, restart estimators
    public void initializeVisionPose(Pose2d pose) {
        m_pose = pose;
        old_pose = pose;
        // reset estimators, uses m_pose just set
        initializeEstimator();
        // keep our vision pose estimators up to date
        if (limelight != null) {
            limelight.setInitialPose(pose, 0.0);
        }
    }

    public Pose2d getVisionPose() {
        return llPose;
    }

    public void printVisionPose() {
        System.out.println("***VisionPose X:" + llPose.getX() +
            ", Y:" + llPose.getY() + ", Rot:" + llPose.getRotation().getDegrees());
    }

   
    public Pose2d getLLEstimate() {
        return llPose;
    }

    public Pose2d getPVEstimate() {
        return pvPose;
    }

    public double getDistanceToTranslation(Translation2d targetTranslation) {
        // TODO - should this be sdt or llpose?
        return Math.sqrt(
                Math.pow(sdt.getPose().getTranslation().getX() - targetTranslation.getX(), 2.0)
                        + Math.pow(sdt.getPose().getTranslation().getY() - targetTranslation.getY(), 2.0));
    }

     /**************************************
     * // bearing should be to or from *what*? split out?
     * public double getBearing() {
     *  return filteredBearing;
     * }
     * 
     * public double getVelocity() {
     *  return filteredVelocity;
     * }
     ************************************/

    /*
     * Watcher for SwervePoseEstimator and its vision data.
     *
     * Only watches high level data, for module details see the tables for each of
     * the modules.
     */
    public class VisionPoseEstimatorMonitorCmd extends WatcherCmd {

        // final private NetworkTable table;
        NetworkTableEntry nt_x_diff;
        NetworkTableEntry nt_y_diff;
        NetworkTableEntry nt_yaw_diff;

        NetworkTableEntry est_ll_pose_x;
        NetworkTableEntry est_ll_pose_y;
        NetworkTableEntry est_ll_pose_h;
        NetworkTableEntry est_pv_pose_x;
        NetworkTableEntry est_pv_pose_y;
        NetworkTableEntry est_pv_pose_h;

        Pose2d ll_pose;
        Pose2d pv_pose;

        public VisionPoseEstimatorMonitorCmd() {
        }

        @Override
        public String getTableName() {
            return VisionPoseEstimator.class.getSimpleName();
        }

        @Override
        public void ntcreate() {
            NetworkTable MonitorTable = getTable();
            est_ll_pose_x = MonitorTable.getEntry("LL_x");
            est_ll_pose_y = MonitorTable.getEntry("LL_y");
            est_ll_pose_h = MonitorTable.getEntry("LL_h");

            est_pv_pose_x = MonitorTable.getEntry("PV_x");
            est_pv_pose_y = MonitorTable.getEntry("PV_y");
            est_pv_pose_h = MonitorTable.getEntry("PV_h");

            // Network Table setup
            nt_x_diff = MonitorTable.getEntry("vision_x_diff");
            nt_y_diff = MonitorTable.getEntry("vision_y_diff");
            nt_yaw_diff = MonitorTable.getEntry("vision_yaw_diff");
        }

        // Network Table Monitoring
        @Override
        public void ntupdate() {
            SmartDashboard.putData("Field_vision", m_field);

            ll_pose = getLLEstimate();
            pv_pose = getPVEstimate();

            if (ll_pose != null) {
                est_ll_pose_x.setDouble(ll_pose.getX());
                est_ll_pose_y.setDouble(ll_pose.getY());
                est_ll_pose_h.setDouble(ll_pose.getRotation().getDegrees());
            }
            if (pv_pose != null) {
                est_pv_pose_x.setDouble(pv_pose.getX());
                est_pv_pose_y.setDouble(pv_pose.getY());
                est_pv_pose_h.setDouble(pv_pose.getRotation().getDegrees());
            }

            // vision pose updating NTs
            nt_x_diff.setDouble(x_diff);
            nt_y_diff.setDouble(y_diff);
            nt_yaw_diff.setDouble(yaw_diff);
        }

    } // monitor cmd class
}
