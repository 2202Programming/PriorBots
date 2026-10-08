// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import static frc.robot2026.Constants.Vision.kMultiTagStdDevs;
import static frc.robot2026.Constants.Vision.kSingleTagStdDevs;
import static frc.robot2026.Constants.Vision.kTagLayout;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.Robot;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.OdometryInterface;
//import frc.lib2202.command.pathing.runPathResetStart;
import frc.robot2026.command.pathing.goDistance;
import frc.robot2026.command.pathing.runPath;
import frc.robot2026.command.pose.setGyroOffsetWithVision;
import frc.robot2026.util.PoseUpdate;

public class Photonvision extends SubsystemBase {

  // individual photonvision USB cameras
  public static class RobotCamera {
    final PhotonCamera camera;
    List<PhotonPipelineResult> results;
    PhotonPipelineResult lastResult;
    boolean hasTargets;
    boolean multiTag;
    List<PhotonTrackedTarget> targets;
    private final PhotonPoseEstimator photonEstimator;
    Pose2d currentPose;
    private Matrix<N3, N1> curStdDevs;
    private double timeStamp;
    ArrayDeque<PoseUpdate> poseUpdateList;

    public RobotCamera(String name, Transform3d kRobotToCam) {
      camera = new PhotonCamera(name);
      photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
      poseUpdateList = new ArrayDeque<PoseUpdate>();

      // Quiet the spam on missing Processors - comment out when debugging PV.
      PhotonCamera.setVersionCheckEnabled(false);
    }

    public void update() {
      targets = null;
      // This method will be called once per scheduler run
      // Query the latest result from PhotonVision
      results = camera.getAllUnreadResults(); // docs say this is preferred, other call deprecated. Only call this once
                                              // per update loop

      // This section looks at most recent photonpipeline result and counts the # of
      // targets seen
      int lastIdx = results.size();
      if (lastIdx > 0) {
        lastResult = results.get(lastIdx - 1);
        hasTargets = lastResult.hasTargets();
        // Get a list of currently tracked targets.
        targets = lastResult.getTargets();
      }

      // @Jason, I think currentPose should be set null at start of update,
      // it should prevent adding old estimates. Same for estStdDevs?
      // @DL I think getAllUnreadResults will be zero length if there are no new
      // pipeline results, so it shouldn't reprocess old frames

      // This section will go through each unread result and generate a pose and
      // timestamp pair and update the pose estimator.
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var result : results) {
        multiTag = true;
        visionEst = photonEstimator.estimateCoprocMultiTagPose(result); // multag if available
        if (visionEst.isEmpty()) { // less than 2 tags, no multitag available
          multiTag = false;
          // Single tag gives a bad pose est.
          // if (result.hasTargets()) {
          // if (result.getBestTarget().getPoseAmbiguity() < 0.1) { // reject single
          // target estimates with high ambiguity
          // visionEst = photonEstimator.estimateLowestAmbiguityPose(result); // use
          // single tag estimator
          // }
          // }
        }

        // this section for updating std dev of results - probably not useful without
        // experimental confirmation of error matrix in constants.
        updateEstimationStdDevs(visionEst, result.getTargets());

        // if visionest is not empty it must mean there was at least one tag in the
        // pipeline result so worth updating currentpose.
        visionEst.ifPresent(
            est -> {
              currentPose = est.estimatedPose.toPose2d();
              var meta = result.metadata.getLatencyMillis();
              timeStamp = est.timestampSeconds - meta / 1000.0;
              poseUpdateList.add(new PoseUpdate(currentPose, timeStamp));

              @SuppressWarnings("unused")
              var estStdDevs = getEstimationStdDevs();
            });
      }
    }

    public double getTimeStamp() {
      return timeStamp;
    }

    public boolean hasATarget() {
      return hasTargets;
    }

    public int howManyTargets() {
      if (targets == null)
        return -1; // targets seems like it can be null, protect - dpl
      return targets.size();
    }

    public PoseUpdate popOldestPoseUpdate() {
      return poseUpdateList.pollFirst();
    }

    public int getPoseUpdateSize() {
      return poseUpdateList.size();
    }

    public Boolean havePose() {
      return (currentPose != null);
    }

    public Pose2d getPose2d() {
      return currentPose;
    }

    // not sure if we should return -1 or 0 if currentPose is null, thoughts @JR
    // this sort of feels wrong to me, maybe use an defaulted Pose2d (zeros) and
    // skip the null test??

    public double getCurrentPoseX() {
      if (currentPose == null)
        return -1;
      return currentPose.getX();
    }

    public double getCurrentPoseY() {
      if (currentPose == null)
        return -1;
      return currentPose.getY();
    }

    public double getCurrentPoseHeading() {
      if (currentPose == null)
        return -1;
      return currentPose.getRotation().getDegrees();
    }

    public boolean hasMultitarget() {
      return multiTag;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
      return curStdDevs;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
            continue;
          numTags++;
          avgDist += tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }
        // TODO if we use the stdev from here, they should get passed along with its
        // PoseUpdate
        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = kSingleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
            estStdDevs = kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
        }
      }
    }
  }

  // Helper Config class to setup camera names and locations for Photonvision
  // Subsystem
  public static class Config {
    // photonvision camera names (needs to match photonvision UI naming)
    public String[] CAMERA_NAMES;

    // Robot to camera transforms.
    // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
    public Transform3d[] kRobotToCam;

    public Config(String[] CAMERA_NAMES, Transform3d[] kRobotToCam) {
      this.CAMERA_NAMES = CAMERA_NAMES;
      this.kRobotToCam = kRobotToCam;
    }
  }

  /** Creates a new Photonvision. */
  final List<RobotCamera> camerasList = new ArrayList<RobotCamera>();
  final Photonvision.Config config;
  List<PoseUpdate> latest_updates; // keep pointer to last collected updates

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;
  private Optional<EstimatedRobotPose> visionEstSim = Optional.empty();
  private final OdometryInterface odo;

  public Photonvision(Config specs) {
    setName("photonvision");
    config = specs;

    for (int i = 0; i < config.CAMERA_NAMES.length; i++) {
      camerasList.add(new RobotCamera(config.CAMERA_NAMES[i], config.kRobotToCam[i]));
    }
    getWatcherCmd();
    odo = RobotContainer.getSubsystemOrNull("odometry");
    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the
      // field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this
      // simulated field.
      visionSim.addAprilTags(kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual
      // camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values
      // with visible
      // targets.
      // Add the simulated camera to view the targets on this simulated field.
      for (int i = 0; i < config.CAMERA_NAMES.length; i++) {
        System.out.println("***Creating PV Sim camera: " + config.CAMERA_NAMES[i]);
        PhotonCameraSim cameraSim = new PhotonCameraSim(camerasList.get(i).camera, cameraProp);
        visionSim.addCamera(cameraSim, config.kRobotToCam[i]);
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
      }
    }
  }

  @Override
  public void periodic() {
    for (RobotCamera currentCamera : camerasList) {
      currentCamera.update(); // run each camera's periodic
    }
    if (Robot.isSimulation()) {
      // visionEstSim.ifPresentOrElse(
      // est -> getSimDebugField()
      // .getObject("VisionEstimation")
      // .setPose(est.estimatedPose.toPose2d()),
      // () -> {
      // getSimDebugField().getObject("VisionEstimation").setPoses();
      // });
      // Update with the simulated drivetrain pose. This should be called every loop
      // in simulation.
      visionSim.update(odo.getPose());
      // Get the built-in Field2d used by this VisionSystemSim
      visionSim.getDebugField();

    }

  }

  // build list of all the updates we have, called by VPE or other estimator
  public List<PoseUpdate> getAllUpdates() {
    List<PoseUpdate> updates = new ArrayList<PoseUpdate>();
    for (RobotCamera cam : camerasList) {
      while (cam.getPoseUpdateSize() > 0) {
        updates.add(cam.popOldestPoseUpdate());
      }
    }
    return updates;
  }

  public int howManyCameras() {
    return camerasList.size(); // config.CAMERA_NAMES.length;
  }

  // some commands can work with the cameras directly
  public List<RobotCamera> getCameras() {
    return camerasList;
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new PhotonWatcher();
  }

  public int totalTargetsAllCameras() {
    int totalTargets = 0;
    for (RobotCamera c : camerasList) {
      totalTargets += c.howManyTargets();
    }
    return totalTargets;
  }

  public boolean anyMultiTags() {
    boolean anyMultiTags = false;
    for (RobotCamera c : camerasList) {
      if (c.hasMultitarget()) {
        anyMultiTags = true;
        break; // finished, exit loop
      }
    }
    return anyMultiTags;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation())
      visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation())
      return null;
    return visionSim.getDebugField();
  }

  // average rotation in mod180 math, vector/double-angle method
  public Rotation2d getAverageRot() {
    double sumSin = 0.0;
    double sumCos = 0.0;

    for (RobotCamera cam : camerasList) {
      Pose2d pose = cam.getPose2d();
      if (pose == null)
        continue;
      if (cam.howManyTargets() > 0) {
        double radians = pose.getRotation().getRadians();
        double doubled = 2 * radians; // multiply each angle by 2 due to 180deg periodicity, makes opposite directions
                                      // align
        double x = Math.sin(doubled); // every angle becomes a point on unit circle
        double y = Math.cos(doubled);
        sumSin += x; // Add all vectors.
        sumCos += y;
      }
    }
    double avgRadians = Math.atan2(sumSin, sumCos) / 2.0; // direction of single resultant vector, divide by two to get
                                                          // back to modulo 180 space.
    double avgDegrees = Math.toDegrees(avgRadians);

    return new Rotation2d(Math.toRadians(avgDegrees));
  }

  public double getAverageRotDegrees() {
    Rotation2d tempRot = getAverageRot();
    return tempRot.getDegrees();
  }

  /*
   * dpl - better to use the camera object directly, then copy into own arrays
   * 
   * public boolean hasMultitarget(int listPos) {
   * return camerasList.get(listPos).hasMultitarget();
   * }
   * 
   * public double getPoseX(int listPos) {
   * return camerasList.get(listPos).getCurrentPoseX();
   * }
   * 
   * public double getPoseY(int listPos) {
   * return camerasList.get(listPos).getCurrentPoseY();
   * }
   */

  public void setDemoBindings(CommandXboxController xbox) {

    xbox.x().onTrue(new goDistance(1.0));
    xbox.a().onTrue(new setGyroOffsetWithVision());
    xbox.b().onTrue(new runPath("Path1"));
  }

  class PhotonWatcher extends WatcherCmd {
    PhotonWatcher() {
      // math is wrong on avg rot, must use modulo math, not simple averages
      // addEntry("Photon Average Rotation", Photonvision.this::getAverageRotDegrees);
      // use camera name/value
      for (RobotCamera cam : camerasList) {
        addEntry(cam.camera.getName() + "/Targets", cam::howManyTargets);
        addEntry(cam.camera.getName() + "/X", cam::getCurrentPoseX, 2);
        addEntry(cam.camera.getName() + "/Y", cam::getCurrentPoseY, 2);
        addEntry(cam.camera.getName() + "/H", cam::getCurrentPoseHeading, 2);
        addEntry(cam.camera.getName() + "/Multi_Target", cam::hasMultitarget);
        addEntry(cam.camera.getName() + "/HasATarget", cam::hasATarget);
        addEntry(cam.camera.getName() + "/timestamp", cam::getTimeStamp);
      }
    }
  }
}
