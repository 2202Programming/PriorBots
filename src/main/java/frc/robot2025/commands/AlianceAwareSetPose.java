package frc.robot2025.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlianceAwareSetPose extends InstantCommand {
    // keep a list of callbacks because different pose estimators/gyros may need to
    // be reset.
    static Set<Consumer<Pose2d>> callBacks = new HashSet<Consumer<Pose2d>>();
    static Set<Consumer<Rotation2d>> rotCallBacks = new HashSet<Consumer<Rotation2d>>();

    // set of callbacks for systems that need to know when gyro reset is needed
    public static void AddCallback(Consumer<Pose2d> consumer) {
        callBacks.add(consumer);
    }
    public static void AddRotationCallback(Consumer<Rotation2d> consumer) {
        rotCallBacks.add(consumer);
    }

    //starting pose on blue side
    Pose2d pose; // blue, gets transposed to red if needed

    public AlianceAwareSetPose(Pose2d poseBlue) {
        this(poseBlue, (Consumer<Pose2d>[]) null);
    }

    @SuppressWarnings("unchecked")
    public AlianceAwareSetPose(Pose2d poseBlue, Consumer<Pose2d>... consumers) {
        this.pose = poseBlue;
        if (consumers != null) {
            for (Consumer<Pose2d> consumer : consumers) {
                AddCallback(consumer);
            }
        }
    }

    @Override
    public void initialize() {
        Pose2d alliancePose;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliancePose = pose;
        } else {
            // red, need to flip, static internal to FlippingUtil determines mirror or rotation
            alliancePose = FlippingUtil.flipFieldPose(pose);
        }
        // call everyone that cares about pose2d reset
        for (Consumer<Pose2d> consumer : callBacks) {
            consumer.accept(alliancePose);
        }

        // call everyone that cares about pose2d reset, like gyros
        for (Consumer<Rotation2d> consumer : rotCallBacks) {
            consumer.accept(alliancePose.getRotation());
        }
        // print what we were set too
        DriverStation.reportWarning("*** AWSetPose() " + alliancePose.toString(), false);
    }
}
