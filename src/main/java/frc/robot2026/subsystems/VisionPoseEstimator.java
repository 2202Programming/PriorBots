package frc.robot2026.subsystems;

import static frc.lib2202.Constants.DEGperRAD;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.subsystem.ILimelight;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.util.VisionWatchdog;
import frc.robot2026.util.PoseUpdate;

// Swerve Drive Train (drivetrain) must be created before Swerve-PoseEstimator

public class VisionPoseEstimator extends SubsystemBase implements OdometryInterface {
    // set true if we found everything needed, otherwise this system is disabled
    final boolean correct_config;

    // This connects us to whatever gyro is being used for robot heading, configured
    // in RobotSpecs
    final IHeadingProvider gyro;
    final DriveTrainInterface drivetrain;
    final OdometryInterface m_odometry; // read-only here, updated in drivetrain
    final SwerveDriveKinematics kinematics; // const matrix based on chassis geometry, get from drivetrain
    SwerveModulePosition[] meas_pos; // provided by drivetrain

    Pose2d m_odoPose; // based on odometry, mechanical wheel counts only

    final VisionWatchdog watchdog;
    final ILimelight limelight;
    final Photonvision photon;

    // stddev based on distance/quality of tag
    final Matrix<N3, N1> closeStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(3.0));
    final Matrix<N3, N1> medStdDevs =VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10.0));
    // final Matrix<N3, N1> farStdDevs =VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(15.0));
    final Matrix<N3, N1> PVStdDevs = VecBuilder.fill(.5, .5, Units.degreesToRadians(10));

    final SwerveDrivePoseEstimator m_estimator;
    // monitor diffs in ll and odometry poses
    private double x_diff; // [m]
    private double y_diff; // [m]
    private double yaw_diff; // [deg]
    private double bot_vel;
    private boolean llValid = false;
    private boolean gyroOffsetMultiTagDone = false;
    private boolean gyroOffsetSingleTagDone = false;

    // vision systems limelight and photonvision(TBD)
    private Pose2d llPose;// latest vision pose (ll and pv)
    private Pose2d prev_llPose;
    private Pose2d rawLLPose;
    private boolean llHasMultitarget;

    //parameters 
    double pCloseDist = 2.0; // [m]  less than this use close std, otherwise use med.

    // field estimate based on vision estimate llPose
    final Field2d m_field;
    final FieldObject2d m_field_obj;
    final String m_ll_name;
    String altName;

    // no-args ctor, default timings
    public VisionPoseEstimator() {
        this(3.0, "limelight"); // typical settings
    }

    // no-args ctor, default timings
    public VisionPoseEstimator(String limelightName) {
        this(0.0, limelightName); // typical settings
    }

    public VisionPoseEstimator(double watchdog_interval, String limelightName) {
        //allow skipping watchdog
        watchdog = (watchdog_interval > 0.0) ? new VisionWatchdog(watchdog_interval) : null;
        m_field = new Field2d();
        m_ll_name = limelightName;
        m_field_obj = m_field.getObject("VPE_odo" + m_ll_name);
        rawLLPose = new Pose2d();
        // other subsystems
        drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
        m_odometry = RobotContainer.getSubsystemOrNull("odometry");
        gyro = RobotContainer.getRobotSpecs().getHeadingProvider();
        limelight = RobotContainer.getSubsystemOrNull(limelightName);       
        photon = RobotContainer.getObjectOrNull("photonvision");

        altName = limelight.getLLName(); // debug

        // new for v2
        limelight.setUseIMU(false);
        limelight.setUse_MT1(false);
        limelight.setUse_MT2(true);

        // confirm config is correct
        correct_config = drivetrain != null && gyro != null &&
                limelight != null && m_odometry != null;

        if (drivetrain != null && m_odometry != null) {
            kinematics = drivetrain.getKinematics();
            meas_pos = drivetrain.getSwerveModulePositions();
            m_odoPose = m_odometry.getPose();
        } else {
            // no drivetrain, set the drivetrain related final vars
            kinematics = null;
            m_odoPose = new Pose2d();
            meas_pos = null;
        }

        // set initial values to odometry based m_odoPose
        llPose = prev_llPose = m_odoPose;

        if (correct_config) {
            // Estimators
            m_estimator = initializeEstimator();
            // start the network monitor
            this.new VisionPoseEstimatorMonitorCmd();
        } else {
            m_estimator = null;
        }

        SmartDashboard.putData("FieldVPE", m_field);
    } // ctor

    @Override
    public void periodic() {
        if (!correct_config)
            return;

        m_odoPose = m_odometry.getPose();
        meas_pos = drivetrain.getSwerveModulePositions();
        llPose = updateEstimator();

        // if we aren't moving and llValid, set m_odometry to use llPose
        if (llValid && bot_vel <= 0.05) {
            // tracking comapare, resync odometry xy, keeps gyro
            m_odometry.setTranslation(llPose.getTranslation());
            m_odoPose = m_odometry.getPose();
        }
        // update field objects
        m_field.setRobotPose(llPose);
        m_field_obj.setPose(m_odoPose);

        if (watchdog != null)
            watchdog.update(rawLLPose, prev_llPose);

        // compare llPose and odometry pose for reporting
        x_diff = (llPose.getX() - m_odoPose.getX());
        y_diff = (llPose.getY() - m_odoPose.getY());
        yaw_diff = (llPose.getRotation().getDegrees() - m_odoPose.getRotation().getDegrees());
    }


    // helper functions
    SwerveDrivePoseEstimator initializeEstimator() {
        /*
         * Here we create SwerveDrivePoseEstimator so that we can fuse odometry
         * readings.
         * The numbers used below are robot specific, and should be tuned.
         * 
         * TODO - add PID config to RobotSpecs
         * TODO - std seem really high for vision, esp the heading
         */
        var estimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                this.meas_pos,
                this.m_odoPose,
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(2)), // std x,y, heading from odmetry [m,deg] 5
                VecBuilder.fill(0.35, 0.35, Units.degreesToRadians(10))); // std x, y heading from vision [m, deg] 30
        return estimator;
    }

    /** Updates the field relative position of the robot. */
    Pose2d updateEstimator() {
        LimelightHelpers.PoseEstimate mt2; // access full mt2 obj for distance to tag
        double dist2Tag = 999.0; // way out, incase no tag.
        prev_llPose = llPose;
        llValid = false; // true on !rejectUpdate
        mt2 = limelight.getMt2();
        // let limelight sub-system decide if we are good to use estimate
        // OK if it is run only intermittantly. Uses latency of vision pose.
        if (mt2 != null && !limelight.getRejectUpdate()) {
            llValid = true;

            var pose = mt2.pose;
            rawLLPose = pose; // for reporting
            double ts = mt2.timestampSeconds;
            dist2Tag = mt2.avgTagDist;

            // speeds in robot-coords
            var bot_speeds = drivetrain.getChassisSpeeds();
            bot_vel = Math.hypot(bot_speeds.vxMetersPerSecond, bot_speeds.vyMetersPerSecond);

            // use sped/dist to weight
            Matrix<N3, N1> stdDev = getStdDev(bot_vel, dist2Tag);

            m_estimator.setVisionMeasurementStdDevs(stdDev);
            m_estimator.addVisionMeasurement(pose, ts);                    
        }

        processPhotonVision();

        // llPose calc - adds heading and drivetrain measurements
        return m_estimator.update(gyro.getRotation2d(), meas_pos);
    }

    // PhotonVision builds list of updates, we grab it here and apply it
    void processPhotonVision() {
        if (photon == null)
            return;

        var updates = photon.getAllUpdates();
        for (PoseUpdate update : updates) {
            m_estimator.addVisionMeasurement(update.pose, update.timestamp, PVStdDevs);
        }
    }

    public void configureGyroCallback() {
        AllianceAwareGyroReset.AddRotationCallback(this::setAnglePose);
    }

    public void setGyroSingleTagDone() {
        gyroOffsetSingleTagDone = true;
    }

    public boolean hasGryoResetSingleTagHappened() {
        return gyroOffsetSingleTagDone;
    }

    public void setGyroMultiTagDone() {
        gyroOffsetMultiTagDone = true;
    }

    public boolean hasGryoResetMultiTagHappened() {
        return gyroOffsetMultiTagDone;
    }


    // see if we can get the LL stddevs for mt1[0..5] and mt2[6..11]
    double[] default_stddevs = new double[12];

    public double[] getStddevs() {
        return NetworkTableInstance.getDefault().getTable(m_ll_name)
                .getEntry("stddevs").getDoubleArray(default_stddevs);
    }

    // Public API
    public void printVisionPose() {
        System.out.println("***VisionPose\n  X:" + llPose.getX() +
                "\n  Y:" + llPose.getY() + "\n  Rot:" + llPose.getRotation().getDegrees());
    }

    public double getDistanceToTranslation(Translation2d targetTranslation) {
        double x_diff = llPose.getX() - targetTranslation.getX();
        double y_diff = llPose.getY() - targetTranslation.getY();
        return Math.hypot(x_diff, y_diff);
        //Math.sqrt(Math.pow(llPose.getX() - targetTranslation.getX(), 2.0) +
        //          Math.pow(llPose.getY() - targetTranslation.getY(), 2.0));
    }

    public double getHeadingToTranslation(Translation2d targetTranslation){
        double x_diff = llPose.getX() - targetTranslation.getX();
        double y_diff = llPose.getY() - targetTranslation.getY();
        return DEGperRAD*Math.atan2(y_diff, x_diff);
    }

    @Override
    public void setPose(Pose2d newPose) {
        // reset gyro, llPose, and odo_pose to the given newPose
        m_odoPose = newPose;
        // set everything to new pose, gyro & odometry
        gyro.setHeading(m_odoPose.getRotation());
        m_odometry.setPose(m_odoPose); // resets drivetrain positions
        // drive positions were cleared, re-read them
        meas_pos = drivetrain.getSwerveModulePositions();
        // set our estimator's newPose with current drivetrains wheel meas_pos
        m_estimator.resetPosition(gyro.getHeading(), meas_pos, m_odoPose);
        llPose = m_estimator.getEstimatedPosition();
    }

    @Override
    public void autoSetPose(Pose2d initialPose) {
        // this will get called if pathplanner resetOdometry is set, we want to ignore
        // if we have a multitarget
        if (!hasGryoResetMultiTagHappened()) {
            setPose(initialPose);// take PP value and hope drive team placed robot well...
            System.out.println("---->Using PathPlanner initial position<----");
        }
        else {
            System.out.println("---->Vision set initial positon----->");
        }
    }

    @Override
    public void setAnglePose(Rotation2d rot) {
        // keep xy, update rotation and gyro
        setPose(new Pose2d(llPose.getTranslation(), rot));
    }

    @Override
    public void setTranslation(Translation2d newPosition) {
        // update the xy, but keeps gyro unchanged
        setPose(new Pose2d(newPosition, gyro.getHeading()));
    }

    @Override
    public Pose2d getPose() {
        return llPose;
    }

    public double getRotationDegrees() {
        return llPose.getRotation().getDegrees();
    }

    @Override
    public void printPose() {
        System.out.println("***VisionPoseEstimator " + m_ll_name +
                "\n   X: " + llPose.getX() +
                "\n   Y: " + llPose.getY() +
                "\n Rot: " + llPose.getRotation().getDegrees());
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Command getWatcher() {
        return this.new VisionPoseEstimatorMonitorCmd();
    }

    // parameters for tuning /w smart dashboard
    double getCloseDistance() {return pCloseDist;}
    void setCloseDistance(double value ) {pCloseDist = value;}


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("CloseDistance m", this::getCloseDistance, this::setCloseDistance);
    }

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
        NetworkTableEntry ll_has_target;
        NetworkTableEntry ll_has_multitarget;
        NetworkTableEntry gyroResetMultiTagDone;
        NetworkTableEntry gyroResetSingleTagDone;
        NetworkTableEntry est_VPE_pose_x;
        NetworkTableEntry est_VPE_pose_y;
        NetworkTableEntry est_VPE_pose_h;

        private final Field2d field;

        public VisionPoseEstimatorMonitorCmd() {
            addEntry("VPE Rotation", VisionPoseEstimator.this::getRotationDegrees);
            addEntry("Vision Gryo Multitag Correction Done", VisionPoseEstimator.this::hasGryoResetMultiTagHappened);
            addEntry("Vision Gryo Singletag Correction Done", VisionPoseEstimator.this::hasGryoResetSingleTagHappened);
            field = new Field2d();
            SmartDashboard.putData("PathWatcher", field);
            field.setRobotPose(llPose);
            field.getObject("target pose").setPose(llPose);

            // Logging callback for current robot pose
            PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                // Do whatever you want with the pose here
                field.setRobotPose(pose);
            });

            // Logging callback for target robot pose
            PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                // Do whatever you want with the pose here
                field.getObject("target pose").setPose(pose);
            });

            // Logging callback for the active path, this is sent as a list of poses
            PathPlannerLogging.setLogActivePathCallback((poses) -> {
                // Do whatever you want with the poses here
                field.getObject("path").setPoses(poses);
            });
        }

        @Override
        public String getTableName() {
            return VisionPoseEstimator.class.getSimpleName();
        }

        @Override
        public void ntcreate() {
            super.ntcreate();
            NetworkTable MonitorTable = getTable();
            est_ll_pose_x = MonitorTable.getEntry("LL/X");
            est_ll_pose_y = MonitorTable.getEntry("LL/Y");
            est_ll_pose_h = MonitorTable.getEntry("LL/Heading");
            ll_has_target = MonitorTable.getEntry("LL/hasATarget");
            ll_has_multitarget = MonitorTable.getEntry("LL/hasMultitarget");

            est_VPE_pose_x = MonitorTable.getEntry("VPE/X");
            est_VPE_pose_y = MonitorTable.getEntry("VPE/Y");
            est_VPE_pose_h = MonitorTable.getEntry("VPE/Heading");

            // Network Table setup
            nt_x_diff = MonitorTable.getEntry("compareLLOdo/diffX");
            nt_y_diff = MonitorTable.getEntry("compareLLOdo/diffY");
            nt_yaw_diff = MonitorTable.getEntry("compareLLOdo/diffHeading");
        }

        // Network Table Monitoring
        @Override
        public void ntupdate() {
            super.ntupdate();
            // The pose just from LL
            if (llPose != null) {
                est_ll_pose_x.setDouble(rawLLPose.getX());
                est_ll_pose_y.setDouble(rawLLPose.getY());
                est_ll_pose_h.setDouble(rawLLPose.getRotation().getDegrees());
                ll_has_target.setBoolean(limelight.getTargetValid());
                if (limelight.getTargetTags() != null) {
                    llHasMultitarget = false;
                    if (limelight.getTargetTags().length > 1) {
                        llHasMultitarget = true;
                    }
                    ll_has_multitarget.setBoolean(llHasMultitarget);
                }
            }

            // the pose from the entire VPE
            est_VPE_pose_x.setDouble(llPose.getX());
            est_VPE_pose_y.setDouble(llPose.getY());
            est_VPE_pose_h.setDouble(llPose.getRotation().getDegrees());

            // vision pose updating NTs
            nt_x_diff.setDouble(x_diff);
            nt_y_diff.setDouble(y_diff);
            nt_yaw_diff.setDouble(yaw_diff);
        }

    } // monitor cmd class

    Matrix<N3, N1> getStdDev(double botvel, double distance) {
        if (distance < pCloseDist)
            return closeStdDevs;

        return medStdDevs;

        // not moving, rank this higher
        // if (botvel < 0.1)
        // return closeStdDevs;
        // if ( distance < 0.5)
        // return closeStdDevs;
        // if (distance < 2.0)
        // return medStdDevs;
        // return farStdDevs;
        // use TBD to pick the stddev to log the vision estimate with
        // return medStdDevs;
    }
}
