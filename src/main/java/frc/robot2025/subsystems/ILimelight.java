package frc.robot2025.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.LimelightHelpers.IMUData;
import frc.robot2025.subsystems.LimelightV2.Retro;

public interface ILimelight {

    // field and tags
    public AprilTagFieldLayout getField();
    public void setField(AprilTagFieldLayout field);
    public int[] getTargetTags();
    public int[] setTargetTags(int[] tag_ids);
    public int[] resetTargetTags();

    // LL modes - retro or MT1/2
    public void setUseRetro(boolean use_retro);     // switches to retro mode
    public boolean getUseRetro();
    public boolean getUse_MT1();
    public void setUse_MT1(boolean use_mt1);  //switches to apriltag mode
    public boolean getUse_MT2();
    public void setUse_MT2(boolean use_mt1);

    // Access the MT data
    public LimelightHelpers.PoseEstimate getMt1();    
    public LimelightHelpers.PoseEstimate getMt2();
    public void setRobotOrientation(Rotation2d heading);  // must call atleast once, depending on imu_mode
    
    // access reflector & pipeline
    public void setPipeline(int pipe);
    public int getPipeline();
    public int setRetroPipeline(int new_retro_pipe);
    public int setMTPipeline(int new_mt_pipe);
    public void resetDefaultPipelines();
    public void enableLED();
    public void disableLED();
    public boolean getLEDStatus();
    public Retro getRetro();
    public boolean getRetroValid();

    // IMU for LL4
    public void setIMUMode(int mode); // LL4 modes, ignored on older LL.
    public int getIMUMode();
    public void setUseIMU(boolean use_imu);
    public IMUData getIMU();

}
