package frc.robot2025.subsystems;

import java.util.List;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.ILimelight;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.LimelightHelpers.IMUData;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;

public class LimelightV2 extends SubsystemBase implements ILimelight {

    // Field Tags and info
    AprilTagFieldLayout m_field = null;
    int[] m_all_tag_ids = null; // all tags on the field. set on loading field.
    int[] m_target_ids = null; // tag ids to filter on (other tag ignored)

    // IMU
    boolean m_use_imu = false;
    IMUData m_imu = null;

    // LL settings
    int m_imu_mode = 1;
    boolean m_use_mt1 = false; // there are times mt1 may be better, when gyro can't be used.
    boolean m_use_mt2 = true; //
    boolean m_use_retro = false; // enable reflective tape
    int m_pipe = 0;

    // LL outputs retro-reflective
    int m_retro_pipe_default = -1;
    int m_retro_pipe = -1;
    Retro retro;

    // LL outputs MT
    int m_apriltag_pipe_default = -1;
    int m_apriltag_pipe = -1;
    int m_tag_count = 0;

    // April Tag PoseEstimates
    Boolean m_mt1_valid = false;
    Boolean m_mt2_valid = false;
    boolean m_reject_update = false;
    LimelightHelpers.PoseEstimate m_mt1;
    LimelightHelpers.PoseEstimate m_mt2;

    // Requried devices
    final IHeadingProvider m_gyro; // pigeon or other gyro needed for MT2, not MT1
    final String m_name; // name of this LL, needed if multiple LL are used.

    public LimelightV2() {
        this("limelight");
    }

    public LimelightV2(String name) {
        this(name, new Pose3d()); // 0,0,0... not great
    }

    public LimelightV2(String name, Pose3d mount_pt) {
        super("LimelightV2(" + name + ")");
        m_name = name;
        m_gyro = RobotContainer.getRobotSpecs().getHeadingProvider();
        // set camera mount_pt
        var llRot = mount_pt.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(name,
                mount_pt.getX(), mount_pt.getY(), mount_pt.getZ(),
                llRot.getX() * DEGperRAD, llRot.getY() * DEGperRAD, llRot.getZ() * DEGperRAD);

        check_pipelines();

        /*
         * LL4 has IMU, others don't but still support MT2 if you set the gyro
         * mode 0 - ignore LL4's IMU, must use external gyro
         * mode 1 - fuse LL4's IMU with given external gyro when you update it
         * mode 2 - use LL4's IMU for MT2 calc, must still initialize atleast once.
         */
        setIMUMode(m_imu_mode);
        disableLED();
        setUse_MT2(true); 

        // callback when we reset the gyro
        AllianceAwareGyroReset.AddRotationCallback(this::setRobotOrientation);
        // always start the LL watcher
        this.new LimelgihtWatcher();
    }

    void check_pipelines() {
        // dump existing config, get default pipes for retro and mt1/2
        for (int idx = 0; idx < 9; idx++) {
            setPipeline(idx);
            LimelightHelpers.Flush();
            sleep(20); // not sure if we need time to change
            String cfg = LimelightHelpers.getCurrentPipelineType(m_name);
            String msg = String.format("LL %s pipeID = %d  cfg= %s", m_name, idx, cfg);
            System.out.println(msg);

            // pick configured pipelines - configured via webpage
            if (cfg.equals("pipe_color") && m_retro_pipe < 0) {
                m_retro_pipe_default = m_retro_pipe = idx;
            }
            if (cfg.equals("pipe_fiducial") && m_apriltag_pipe < 0) {
                m_apriltag_pipe_default = m_apriltag_pipe = idx;
            }
        }
       
        // report default pipes
        String msg = String.format("LL '%s' apriltag pipeID = %d", m_name, m_apriltag_pipe);
        System.out.println(msg);
        msg = String.format("LL '%s' retro pipeID = %d", m_name, m_retro_pipe);
        System.out.println(msg);

        if (m_apriltag_pipe < 0) {
            msg = String.format("!!!!!! WARNING NO APRILTAG PIPE CONFIGURED for '%s'!!!!", m_name);
            DriverStation.reportWarning(msg, false);
        } else {
            setPipeline(m_apriltag_pipe);
            return;
        }
    }

    @Override
    public void periodic() {
        // read LL values
        m_mt1 = null;
        m_mt2 = null;
        retro = null;
        m_imu = null;
        m_mt1_valid = false;
        m_mt1_valid = false;
        m_reject_update = true;
        boolean mt1_reject_update = false;
        boolean mt2_reject_update = false;

        // mode 1 - needs a regular gyro update
        if (m_imu_mode == 1 && m_gyro != null) {
            setRobotOrientation(m_gyro.getHeading());
        }

        // Retro seem exclusive of apriltags because of pipeline switch needed
        // Reflective Interface
        if (m_use_retro) {
           retro = new Retro(m_name);
        } else {
            // MT1
            if (m_use_mt1) {
                m_mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);                
                m_mt1_valid = LimelightHelpers.validPoseEstimate(m_mt1);
                mt1_reject_update = !m_mt1_valid ||
                                    (m_mt1.rawFiducials[0].ambiguity > 0.7) ||    //mt1 only
                                    (m_mt1.rawFiducials[0].distToCamera > 3.0);   //mt1 only                                      
            }
            // MT2 can be used with MT1
            if (m_use_mt2) {
                m_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);
                m_mt2_valid = LimelightHelpers.validPoseEstimate(m_mt2); 
                mt2_reject_update = !m_mt2_valid;
            }
            // warning if using both mt1&2, a bad mt2 could block a good mt1 or vice-versa
            // if this is a problem, parse the specific mt<n> object in your use.
            m_reject_update = (mt1_reject_update) || 
                              (mt2_reject_update) ||
                              (Math.abs(m_gyro.getYawRate()) > 720.0);
        }
        
        // read IMU 
        if (m_use_imu) {
            m_imu = LimelightHelpers.getIMUData(m_name);
        }
    }

    //name of the LL device
    public String getLLName() {
        return m_name;
    }

    @Override
    public boolean getRejectUpdate() {
        return m_reject_update;
    }

    @Override
    public boolean getTargetValid(){
        return m_mt1_valid || m_mt2_valid;
    }

    // pipelines
    @Override
    public void setPipeline(int pipe) {
        m_pipe = pipe;
        ILimelight.super.setPipeline( m_pipe);
    }

    @Override
    public int getPipeline() {
        int c_pipe = ILimelight.super.getPipeline();
        if (c_pipe != m_pipe) {
            // helpers uses different tables for setting and geting, this tests the behavoir.
            // if this never happens, we can remove the code and use iLimelight impl directly
            System.out.format("LLV2 c_pipe= %d differs from interal m_pipe= %d",c_pipe, m_pipe);
            m_pipe = c_pipe;
        }
        return m_pipe;
    }

    //Override the MT pipeline, set it if mt is active. Caller should save returned pipe id.
    public int setMTPipeline(int new_mt_pipe) {
        int old = m_apriltag_pipe;
        m_apriltag_pipe = new_mt_pipe;
        if (m_use_mt1 || m_use_mt2)
            setPipeline(new_mt_pipe);
        return old;
    } 

    //Override the retro pipeline, set it if mt is active. Caller should save returned pipe id.
    public int setRetroPipeline(int new_retro_pipe) {
        int old = m_retro_pipe;
        m_retro_pipe = new_retro_pipe;
        if (m_use_retro)
            setPipeline(new_retro_pipe);
        return old;
    } 

    public void resetDefaultPipelines() {
        setRetroPipeline(m_retro_pipe_default);
        setMTPipeline(m_apriltag_pipe_default);
    }

    // Retro or Apriltag modes
    public void setUseRetro(boolean use_retro) {
        m_use_retro = use_retro;
        if (m_use_retro) {
            m_use_mt1 = false;  //disable MT usage
            m_use_mt2 = false;
            m_use_retro = use_retro;
            setPipeline(m_retro_pipe);
        }
        else{
            // return to apriltag pipe
            setUse_MT2(true);
        }
    }

    public boolean getUseRetro(){
        return m_use_retro;
    }

    // Access Retro data
    public boolean getRetroValid(){
        return (retro != null) ? retro.tv : false;        
    }

    public Retro getRetro(){
        return retro;
    }

    // MT1
    public boolean getMT1Valid() {
        return m_mt1_valid;
    }

    public LimelightHelpers.PoseEstimate getMt1() {
        return m_mt1;
    }

    // MT2
    public boolean getMT2Valid() {
        return m_mt2_valid;
    }

    public LimelightHelpers.PoseEstimate getMt2() {
        return m_mt2;
    }

    // mt1 & mt1 flags
    public boolean getUse_MT1() {
        return m_use_mt1;
    }

    public void setUse_MT1(boolean use_mt1) {
        m_use_mt1 = use_mt1;
        if (m_use_mt1 || m_use_mt2)
            setPipeline(m_apriltag_pipe);
    }

    public boolean getUse_MT2() {
        return m_use_mt2;
    }

    public void setUse_MT2(boolean use_mt2) {
        m_use_mt2 = use_mt2;
        if (m_use_mt1 || m_use_mt2)
            setPipeline(m_apriltag_pipe);
    }

    // Field and Tags 
    public void setField(AprilTagFieldLayout field) {
        m_field = field;
        var tags = m_field.getTags();
        m_all_tag_ids = setTargetTags(tags); // save all field tags for reset
    }

    public AprilTagFieldLayout getField() {
        return m_field;
    }

    public int[] setTargetTags(List<AprilTag> tags) {
        m_target_ids = new int[tags.size()];
        for (int i = 0; i < tags.size(); i++)
            m_target_ids[i] = tags.get(i).ID;
        LimelightHelpers.SetFiducialIDFiltersOverride(m_name, m_target_ids);
        return m_target_ids;
    }

    public int[] setTargetTags(int[] tag_ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(m_name, tag_ids);
        m_target_ids = tag_ids;
        return m_target_ids;
    }

    public int[] getTargetTags() {
        return m_target_ids;
    }

    public int[] resetTargetTags() {
        if (m_all_tag_ids == null) {
            DriverStation.reportWarning(m_name + " needs a field set to use tags.", false);
            return null;
        }
        setTargetTags(m_all_tag_ids);
        return m_all_tag_ids;
    }

    // LL4 IMU setting
    //enables reading IMU data in the periodic m
    public void setUseIMU(boolean use_imu){
        m_use_imu = use_imu;
    }

    public boolean getUseIMU(){
        return m_use_imu;
    }

    public IMUData getIMU(){
        return m_imu;
    }

    public void setRobotOrientation(Rotation2d heading) {
        LimelightHelpers.SetRobotOrientation(m_name, heading.getDegrees(), 0, 0, 0, 0, 0);
    }

    // Watcher Cmd
    public class LimelgihtWatcher extends WatcherCmd {

        // NT entries and SmartDashboard object
        NetworkTableEntry nt_foo;

        @Override
        public void ntcreate() {

        }

        @Override
        public void ntupdate() {

        }

    }


    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
    }

}
