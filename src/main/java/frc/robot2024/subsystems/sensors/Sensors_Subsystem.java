/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// NOTE: DPL replaced with sensors ss used in 2025 which had improvement on seting gyro and matched interface.
package frc.robot2024.subsystems.sensors;

import static frc.lib2202.Constants.DEGperRAD;
import static frc.lib2202.Constants.DT;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.util.ModMath;
import frc.robot2024.Constants.CAN;

public class Sensors_Subsystem extends SubsystemBase implements IHeadingProvider {
  /**
   * Creates a new Sensors_Subsystem.
   * 
   * This class will collect various robot sensors and ensure they are sampled and
   * filtered together.
   * 
   * 
   */

  final int NT_UPDATE_FRAME = 10; 
  int log_counter = 0;

  private NetworkTable table;
  //CAN
  private NetworkTableEntry nt_canUtilization;
  private NetworkTableEntry nt_canTxError;
  private NetworkTableEntry nt_canRxError;
  // angles
  private NetworkTableEntry nt_yaw;
  private NetworkTableEntry nt_yaw_offset;
  private NetworkTableEntry nt_yaw_Z;
  private NetworkTableEntry nt_yaw_dot;
  private NetworkTableEntry nt_roll;
  private NetworkTableEntry nt_roll_dot;
  private NetworkTableEntry nt_pitch;
  private NetworkTableEntry nt_pitch_dot;
  private NetworkTableEntry nt_rotation;

  // Sensors & CAN monitoring
  Pigeon2 m_pigeon;
  CANStatus m_canStatus;

  // Simulation TBD
  boolean simInit = false;
  Pigeon2SimState simPigeon;
  
  // measured angles and rates, in degrees
  double m_roll;
  double m_roll_d;
  double m_pitch;
  double m_pitch_d;
  double m_yaw;         //includes offset
  double m_yaw_offset;  //avoid CAN io, so track offset on resetting gyro
  double m_yaw_Z;  //uses pigeon.getYaw, debugging
  double m_yaw_d;
  
  //accelerations
  double m_Xaccel;
  double m_Yaccel;
  double m_Zaccel;

  //bias offsets measured at power up
  final int BIAS_SAMPLES = 5; // [count]
  final double BIAS_DELAY = 0.2; // [s]
  double m_roll_bias; // [deg]
  double m_pitch_bias; // [deg]
  double m_yaw_bias; // [deg] measured, but not corrected for

  // set this to true to default to pigeon
  public Pose2d autoStartPose;
  public Pose2d autoEndPose;

  public Sensors_Subsystem() {
    // alocate sensors
    m_canStatus = new CANStatus();
    m_pigeon = new Pigeon2(CAN.PIGEON_IMU_CAN);
    m_pigeon.reset();
    m_yaw_offset = 0.0;
    m_pigeon.clearStickyFaults();

    // setup network table
    table = NetworkTableInstance.getDefault().getTable("Sensors");
    nt_canUtilization = table.getEntry("CanUtilization");
    nt_canRxError = table.getEntry("CanRxError");
    nt_canTxError = table.getEntry("CanTxError");

    nt_rotation = table.getEntry("Rotation");
    nt_pitch = table.getEntry("Pitch");
    nt_roll = table.getEntry("Roll");
    nt_yaw = table.getEntry("Yaw");
    nt_yaw_offset = table.getEntry("yawOffset");
    nt_yaw_Z = table.getEntry("yaw_Z");

    nt_yaw_dot = table.getEntry("YawDot");
    nt_pitch_dot = table.getEntry("PitchDot");
    nt_roll_dot = table.getEntry("RollDot");

    calibrate();
    log();
  }

  // @Override
  public void calibrate() {
    double roll_bias = 0.0, pitch_bias = 0.0, yaw_bias = 0.0;
    for (int i = 0; i < BIAS_SAMPLES; i++) {

      // dpl - not sure how much these differ
      // roll_bias += m_pigeon.getRotation3d().getX() * 180.0 / Math.PI;// Roll
      // pitch_bias += m_pigeon.getRotation3d().getY() * 180.0 / Math.PI;// Pitch
      // yaw_bias += m_pigeon.getRotation3d().getZ() * 180.0 / Math.PI;// Yaw

      // use pigeon's direct r/p/y
      roll_bias += m_pigeon.getRoll().getValueAsDouble();
      pitch_bias += m_pigeon.getPitch().getValueAsDouble();
      yaw_bias += m_pigeon.getYaw().getValueAsDouble();
      Timer.delay(BIAS_DELAY);
    }
    // save bias value to subtract from live measurements
    m_roll_bias = roll_bias / (double) BIAS_SAMPLES;
    m_pitch_bias = pitch_bias / (double) BIAS_SAMPLES;
    m_yaw_bias = yaw_bias / (double) BIAS_SAMPLES;

    System.out.println("\t\tSensors measured yaw_bias= " + m_yaw_bias );
    System.out.println("\t\tSensors measured pitch_bias= " + m_pitch_bias );
    System.out.println("\t\tSensors measured roll_bias= " + m_roll_bias );
  }

  @Override
  public void periodic() {
    // CCW positive
    m_yaw_Z = ModMath.fmod360_2(m_pigeon.getRotation3d().getZ() * 180.0 / Math.PI
            + m_yaw_offset); // quaternian-based works in field centric
    // m_pitch = (m_pigeon.getRotation3d().getY() * 180.0 / Math.PI) - m_pitch_bias;
    // m_roll = (m_pigeon.getRotation3d().getX() * 180.0 / Math.PI) - m_roll_bias;

    // pigeon gets done with (false) so as not to wait, using cached value

    // Use the direct r/p/y from pigeon instead of above unpack from 3d ... not sure of difference 
    m_roll = m_pigeon.getRoll(false).getValueAsDouble() - m_roll_bias;
    m_pitch = m_pigeon.getPitch(false).getValueAsDouble() - m_pitch_bias;

    // this wasn't working in FieldCentric, had -m_pigeon.getYaw(), could be sign...?
    m_yaw = ModMath.fmod360_2(m_pigeon.getYaw(true).getValueAsDouble() 
      + m_yaw_offset); 
    
    // Getting the angular velocities [deg/s]
    m_roll_d = m_pigeon.getAngularVelocityXWorld(false).getValueAsDouble();
    m_pitch_d = m_pigeon.getAngularVelocityYWorld(false).getValueAsDouble();
    m_yaw_d = m_pigeon.getAngularVelocityZWorld(true).getValueAsDouble();
   
    // read accelerations
    m_Xaccel = m_pigeon.getAccelerationX(false).getValueAsDouble();
    m_Yaccel = m_pigeon.getAccelerationY(false).getValueAsDouble();
    m_Zaccel = m_pigeon.getAccelerationZ(false).getValueAsDouble();
    log();
  }

  // use sdt to fake gryo
  DriveTrainInterface sdt = null;

  void setupSimulation() {
    sdt = RobotContainer.getSubsystemOrNull("drivetrain");
    simPigeon = m_pigeon.getSimState();
    simPigeon.setSupplyVoltage(12.0);
    simInit = true;
  }

  @Override
  public void simulationPeriodic() {
    if (!simInit) setupSimulation();    
    if (sdt == null) return;
    var field_speeds = sdt.getFieldRelativeSpeeds();
    var yaw_rate = field_speeds.omegaRadiansPerSecond * DT * DEGperRAD;
    simPigeon.addYaw(yaw_rate);
    simPigeon.setAngularVelocityZ(yaw_rate);
    
  }


  public void log() {
    if ((log_counter++ % NT_UPDATE_FRAME) == 0) {

      CANJNI.getCANStatus(m_canStatus);
      nt_canUtilization.setDouble(m_canStatus.percentBusUtilization);
      nt_canRxError.setNumber(m_canStatus.receiveErrorCount);
      nt_canTxError.setNumber(m_canStatus.transmitErrorCount);

      nt_yaw.setDouble(getYaw());
      nt_yaw_offset.setDouble(m_yaw_offset);
      nt_yaw_Z.setDouble(m_yaw_Z);
  
      //
      nt_rotation.setDouble(getRotation2d().getDegrees());
      nt_roll.setDouble(getRoll());
      nt_pitch.setDouble(getPitch());

      nt_yaw_dot.setDouble(getYawRate());
      nt_roll_dot.setDouble(getRollRate());
      nt_pitch_dot.setDouble(getPitchRate());
    }
  }

  // All accessors return values measured in the periodic()
  public double getRoll() {
    return m_roll;
  }

  public double getPitch() {
    return m_pitch;
  }

  public double getPitchRate() {
    return m_pitch_d;
  }

  public double getRollRate() {
    return m_roll_d;
  }

   /**
   * Return the rate of rotation of the yaw gyro.
   *
   * <p>
   * The rate is based on the most recent reading of the gyro analog value
   *
   * <p>
   * The rate is expected to be positive as the gyro turns clockwise when looked
   * at from the top. It needs to follow the NED axis convention.
   *
   * @return the current yaw rate in degrees per second
   */
  // @Override
  public double getYawRate() {
    return m_yaw_d;
  }

  public double getXAccel() {
    return m_Xaccel;
  }

  public double getYAccel() {
    return m_Yaccel;
  }

  public double getZAccel() {
    return m_Zaccel;
  }

  /*
   * not sure why these would be needed - Mr.L 2//22/2023
   * public double getTotalTilt() {
   * return Math.sqrt(Math.pow(getPitch(), 2) + Math.pow(getRoll(), 2));
   * }
   * 
   * public double getTotalTiltRate() {
   * return Math.sqrt(Math.pow(getPitchRate(), 2) + Math.pow(getRollRate(), 2));
   * }
   */

  /**
   * Return the heading of the robot in degrees.
   *
   * <p>
   * The angle is continuous, that is it will continue from 360 to 361 degrees.
   * This allows algorithms that wouldn't want to see a discontinuity in the gyro
   * output as it sweeps past from 360 to 0 on the second time around.
   *
   * <p>
   * The angle is expected to increase as the gyro turns clockwise when looked at
   * from the top. It needs to follow the NED axis convention.
   *
   * <p>
   * This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot in degrees.
   */
  public double getYaw() {
    return m_yaw;
  }

  /*
   * setYaw() - will do a CAN IO call
   * use setRotation2d() which tracks offset and doesn't use CAN blocking call.
   * 
   */
  @Deprecated
  public void setYaw(double yawDegrees) {
    m_pigeon.setYaw(yawDegrees);
    m_yaw_offset = 0.0;  //no offset needed, gyro state set
    DriverStation.reportError("***DEPRECATED setYaw() <== " + yawDegrees, false);
  }
  @Deprecated
  public void setYaw(Rotation2d rotation) {
    setYaw(rotation.getDegrees());
  }

  /**
   * Return the heading of the robot as a
   * {@link edu.wpi.first.math.geometry.Rotation2d}.
   *
   * <p>
   * The angle is continuous, that is it will continue from 360 to 361 degrees.
   * This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it
   * sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>
   * The angle is expected to increase as the gyro turns counterclockwise when
   * looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * <p>
   * This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot as a 
   *   {@link edu.wpi.first.math.geometry.Rotation2d}.
   */
  // @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_yaw); // note sign
  }

  /*
   * set the gyro to the new value by adjusting the offset, the actual
   * gyro is not changed. This avoids expensive CAN set call. (100ms worst case)
   */
  public void setRotation2d(Rotation2d newrot) {
    double rawYaw = ModMath.fmod360_2(m_yaw - m_yaw_offset); // take out current offset for raw gyro's yaw
    m_yaw = ModMath.fmod360_2(newrot.getDegrees());
    m_yaw_offset = m_yaw - rawYaw;  
  }

  // 2/12/2025 We shouldn't have to mess with pose and setYaw. Odometry seems to handle
  // tracking yaw offsets now.
  // deprcate and see where we still might use these.
  @Deprecated
  public void setAutoStartPose(Pose2d pose) {
    autoStartPose = new Pose2d(pose.getTranslation(), pose.getRotation());
    setYaw(pose.getRotation()); // set gyro to starting heading so it's in field coordinates.
    DriverStation.reportError("***DEPRECATED Auto Start Pose set: " + pose, false);
  }

  @Deprecated
  public void setAutoEndPose(Pose2d pose) {
    autoEndPose = new Pose2d(pose.getTranslation(), pose.getRotation());

    // expected difference in heading from start of auto to end
    Rotation2d autoRot = autoStartPose.getRotation().minus(autoEndPose.getRotation());

    // gyro should power on at zero heading which would be our auto start position's
    // heading. So any angle off zero is the difference from start to end per the
    // gyro
    // not sure if this should be added or subtracted
    Rotation2d rotError = autoRot.minus(Rotation2d.fromDegrees(getYaw()));

    System.out.println("***Auto End Pose set: " + pose);
    System.out.println("***Rotation difference per Pose: " + autoRot.getDegrees());
    System.out.println("***Rotation difference per Gyro: " + getYaw());
    System.out.println("***Difference: " + rotError.getDegrees());

    /*
     * Idea below for correcting pose angle
     * Since before we run each path we set our pose to the starting position,
     * it's possible that our "true" heading (as determined by gryo) is not exactly
     * the starting heading of the new path.
     * The end of the prior path should be the start of the new path, but presumably
     * the rotation is not perfectly aligned (PID errors)
     * So with multiple paths this rotation error in pose may accumulate?
     */
    // RobotContainer.RC().drivetrain.resetAnglePose(pose.getRotation().minus(rotError));
    DriverStation.reportError("***DEPRECATED Auto End Pose set: " + pose, false);
  }
}
