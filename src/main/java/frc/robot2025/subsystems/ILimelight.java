package frc.robot2025.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot2025.subsystems.LimelightV2.Retro;

public interface ILimelight {

    public AprilTagFieldLayout getField();

    public void setField(AprilTagFieldLayout field);

    public int[] getTargetTags();

    public int[] setTargetTags(int[] tag_ids);

    public int[] resetTargetTags();

    public void setUseRetro(boolean use_retro);     // switches to retro mode
    public boolean getUseRetro();

    public boolean getUse_MT1();

    public void setUse_MT1(boolean use_mt1);  //switches to apriltag mode

    public boolean getUse_MT2();

    public void setUse_MT2(boolean use_mt1);


    // access reflector & pipeline
    public void setPipeline(int pipe);
    public int getPipeline();
    public void enableLED();
    public void disableLED();
    public boolean getLEDStatus();
    public Retro getRetro();

    public void setIMUMode(int mode); // LL4 modes, ignored on older LL.

    public int getIMUMode();

}
