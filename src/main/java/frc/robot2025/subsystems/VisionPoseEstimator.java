package frc.robot2025.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.subsystem.BaseLimelight;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.LimelightV1;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.SignalLight;
import frc.lib2202.subsystem.SignalLight.Color;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.util.VisionWatchdog;

// Swerve Drive Train (drivetrain) must be created before Swerve-PoseEstimator

public class VisionPoseEstimator extends SubsystemBase implements OdometryInterface
{
    // set true if we found everything needed, otherwise this system is disabled
    final boolean correct_config;

    // This connects us to whatever gyro is being used for robot heading, configured
    // in RobotSpecs
    final IHeadingProvider gyro;
    final DriveTrainInterface drivetrain;  
    final OdometryInterface m_odometry;    // read-only here, updated in drivetrain
    final SwerveDriveKinematics kinematics; // const matrix based on chassis geometry, get from drivetrain
    SwerveModulePosition[] meas_pos; // provided by drivetrain

    Pose2d m_odoPose;  //based on odometry

    final VisionWatchdog watchdog;
    final BaseLimelight limelight;
    final LimelightV1  ll2025;    //temp way to acces new funcs this year
    final SignalLight signal;
    
    // stddev based on distance/quality of tag
    final Matrix<N3, N1> closeStdDevs =VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2.0));
    final Matrix<N3, N1> medStdDevs =VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(5.0));
    final Matrix<N3, N1> farStdDevs =VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(15.0));
    
    boolean visionPoseUsingRotation = true; // read from drivetrain.useVisionRotation()
    boolean visionPoseEnabled = true;

    final SwerveDrivePoseEstimator m_estimator;
    // monitor diffs in ll and odometry poses
    private double x_diff; // [m]
    private double y_diff; // [m]
    private double yaw_diff; // [deg]
    private double bot_vel;
    private boolean llValid = false;

    //vision systems limelight and photonvision(TBD)
    private Pose2d llPose;
    private Pose2d prev_llPose;

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
        this(3.0, limelightName); // typical settings
    }

    public VisionPoseEstimator(double watchdog_interval, String limelightName) {
        watchdog = new VisionWatchdog(watchdog_interval);
        m_field = new Field2d();
        m_ll_name = limelightName;
        m_field_obj = m_field.getObject("VPE_odo" + m_ll_name);

        // other subsystems
        drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
        m_odometry = RobotContainer.getSubsystemOrNull("odometry");
        gyro = RobotContainer.getRobotSpecs().getHeadingProvider();
        limelight = RobotContainer.getSubsystemOrNull(limelightName);
        signal = RobotContainer.getObjectOrNull("light");

        if (limelight instanceof LimelightV1) {
            ll2025 = (LimelightV1)limelight; //horrible hack
        }
        else ll2025 = null;

        altName = limelight.getName();  //debug

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
        
        //set initial values to odometry based m_odoPose
        llPose = prev_llPose = m_odoPose;
        
        if (correct_config) {
            // Estimators
            m_estimator = initializeEstimator();
            // start the network monitor
            this.new VisionPoseEstimatorMonitorCmd();
        }
        else {
            m_estimator = null;
        }

        SmartDashboard.putData("FieldVPE", m_field );
    } // ctor


    @Override
    public void periodic() {
        if (!correct_config) return;

        m_odoPose = m_odometry.getPose();
        meas_pos = drivetrain.getSwerveModulePositions();
        llPose = updateEstimator();

        // if we aren't moving and llValid, set m_odometry to use llPose
        if (llValid && bot_vel <= 0.05) {
            //tracking comapare, resync odometry xy, keeps gyro
            m_odometry.setTranslation(llPose.getTranslation());
            m_odoPose = m_odometry.getPose();
        }
        // update field objects
        m_field.setRobotPose(llPose);
        m_field_obj.setPose(m_odoPose);

        // compare llPose and odometry pose for reporting       
        x_diff = (llPose.getX() - m_odoPose.getX());
        y_diff = (llPose.getY() - m_odoPose.getY());
        yaw_diff = (llPose.getRotation().getDegrees() - m_odoPose.getRotation().getDegrees());

        SetSignal();
    }

    void SetSignal() {
        if(signal == null) return;

        Color color = SignalLight.Color.RED; // nothing visible

        if (!limelight.getRejectUpdate()) { // true = not valid
            color = SignalLight.Color.GREEN;
            if(bot_vel < 0.1) {
                color = SignalLight.Color.BLUE;
            }
        }
        signal.setLight(color);
    }
    // helper functions
    SwerveDrivePoseEstimator initializeEstimator() {
        /*
         * Here we create SwerveDrivePoseEstimator so that we can fuse odometry readings.
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
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)), // std x,y, heading from odmetry [m,deg]  5
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10))); // std x, y heading from vision [m, deg] 30
        return estimator;
    }

    /** Updates the field relative position of the robot. */
    Pose2d updateEstimator() {
        LimelightHelpers.PoseEstimate mt2;  // access full mt2 obj for distance to tag
        double dist2Tag = 999.0;  //way out, incase no tag.
        prev_llPose = llPose;
        llValid = false;  // true on !rejectUpdate

        // let limelight sub-system decide if we are good to use estimate
        // OK if it is run only intermittantly. Uses latency of vision pose.
        if (!limelight.getRejectUpdate()) { 
            llValid = true;
            var pose = limelight.getBluePose();
            var ts = limelight.getVisionTimestamp();
            if (ll2025 != null) {
                mt2 = ll2025.getMt2();
                dist2Tag = mt2.avgTagDist;
            }
             // speeds in robot-coords
            var bot_speeds = drivetrain.getChassisSpeeds();
            bot_vel = Math.hypot(bot_speeds.vxMetersPerSecond, bot_speeds.vyMetersPerSecond);
            
            //use sped/dist to weight 
            Matrix<N3, N1> stdDev = getStdDev(bot_vel, dist2Tag);

            m_estimator.setVisionMeasurementStdDevs(stdDev);
            m_estimator.addVisionMeasurement(pose, ts);
            if (watchdog != null)
                watchdog.update(pose, prev_llPose);
        }

        //llPose calc - adds heading and drivetrain measurements
        return m_estimator.update(gyro.getRotation2d(), meas_pos);       
    }

    /******************************************
    //set drivetrain's pose if it's enabled
    // This couples odometry by forcing it to take the LL pose.
    // really we are incorporating the odometry measurements into the LLPoseEstimator.
    // Should be able to have pathing users take this estimator's pose
    @Deprecated
    void useEstimate() {
        visionPoseUsingRotation = m_odometry.useVisionRotation();
        visionPoseEnabled = m_odometry.useVisionPose();
        
        Rotation2d current_rotation = m_odoPose.getRotation();
        if (visionPoseEnabled) {           
            if(watchdog != null) watchdog.update(llPose, prev_llPose);
            if (visionPoseUsingRotation) {
                // update robot pose, include vision-based rotation
                m_odometry.setPose(llPose);
            } else {
                // update robot translation, do not update rotation
                m_odometry.setPose(new Pose2d(llPose.getTranslation(), current_rotation));
            }         
        }
    }
    **************************************************/

    public void configureGyroCallback(){
        AllianceAwareGyroReset.AddRotationCallback(this::setAnglePose);
    }
    
    
    // see if we can get the LL stddevs for mt1[0..5] and mt2[6..11]
    double[] default_stddevs = new double[12];
    public double[] getStddevs() {
        return 
            NetworkTableInstance.getDefault().getTable(m_ll_name)
                .getEntry("stddevs").getDoubleArray(default_stddevs);
      }

    /** 
     * @return Pose2d
     */
    // Public API

    public void printVisionPose() {
        System.out.println("***VisionPose\n  X:" + llPose.getX() +
            "\n  Y:" + llPose.getY() + "\n  Rot:" + llPose.getRotation().getDegrees());
    }

    public double getDistanceToTranslation(Translation2d targetTranslation) {
        return Math.sqrt(
            Math.pow(llPose.getX() - targetTranslation.getX(), 2.0) +
            Math.pow(llPose.getY() - targetTranslation.getY(), 2.0));
    }
    
    @Override
    public void setPose(Pose2d newPose) {
        // reset gyro, llPose, and odo_pose to the given newPose
        m_odoPose = newPose;
        // set everything to new pose, gyro & odometry
        gyro.setHeading(m_odoPose.getRotation()); 
        m_odometry.setPose(m_odoPose);
        
        // drive positions could be cleared, re-read them
        meas_pos = drivetrain.getSwerveModulePositions();
        // set our estimator's newPose with current drivetrains wheel meas_pos
        m_estimator.resetPosition(gyro.getHeading(), meas_pos, m_odoPose);
        llPose = m_estimator.getEstimatedPosition(); 
    }
    
    @Override
    public void setAnglePose(Rotation2d rot) {
        //keep xy, update rotation and gyro
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

        private final Field2d field;

        public VisionPoseEstimatorMonitorCmd() {
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
            NetworkTable MonitorTable = getTable();
            est_ll_pose_x = MonitorTable.getEntry("LL/X");
            est_ll_pose_y = MonitorTable.getEntry("LL/Y");
            est_ll_pose_h = MonitorTable.getEntry("LL/Heading");

            // Network Table setup
            nt_x_diff = MonitorTable.getEntry("compareLLOdo/diffX");
            nt_y_diff = MonitorTable.getEntry("compareLLOdo/diffY");
            nt_yaw_diff = MonitorTable.getEntry("compareLLOdo/diffHeading");
        }

        // Network Table Monitoring
        @Override
        public void ntupdate() {           
            if (llPose != null) {
                est_ll_pose_x.setDouble(llPose.getX());
                est_ll_pose_y.setDouble(llPose.getY());
                est_ll_pose_h.setDouble(llPose.getRotation().getDegrees());
            }

            // vision pose updating NTs
            nt_x_diff.setDouble(x_diff);
            nt_y_diff.setDouble(y_diff);
            nt_yaw_diff.setDouble(yaw_diff);
        }

    } // monitor cmd class


    Matrix<N3, N1> getStdDev(double botvel, double distance ) {
        return closeStdDevs;
        // not moving, rank this higher
        // if (botvel < 0.1) 
        //     return closeStdDevs;
        // if ( distance < 0.5)
        //     return closeStdDevs;
        // if (distance < 2.0)
        //     return medStdDevs;
        // return farStdDevs;
        // use TBD to pick the stddev to log the vision estimate with
        //return medStdDevs;
    }
}
