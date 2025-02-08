/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.chadbot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chadbot.Constants.CAN;
import frc.chadbot.Constants.NTStrings;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.util.ModMath;

public class Sensors_Subsystem extends SubsystemBase implements IHeadingProvider{
  /**
   * Creates a new Sensors_Subsystem.
   * 
   * This class will collect various robot sensors and ensure they are sampled and
   * filtered together.
   * 
   * Converted to Phoenix 6, removed CANcoder inits - dpl 11/20/2024
   * Removed NavX related stuff
   * Removed old wip raw pigeon signal code
   */

  final double Kgyro = -1.0; // ccw is positive, just like geometry class
  final double g_mps2 = 9.80665; // [m/s^2/g]

  private NetworkTable table;
  private NetworkTable positionTable;
  // private NetworkTableEntry nt_accelX;
  // private NetworkTableEntry nt_accelY;
  // private NetworkTableEntry nt_accelZ;
  private NetworkTableEntry nt_yaw_mod180;

  private NetworkTableEntry nt_canUtilization;
  private NetworkTableEntry nt_canTxError;
  private NetworkTableEntry nt_canRxError;

  private NetworkTableEntry nt_yaw;
  private NetworkTableEntry nt_roll;
  private NetworkTableEntry nt_pitch;
  private NetworkTableEntry nt_rotation;

  static final byte update_hz = 100;

  // Sensors
  final Pigeon2 m_pigeon;
  // roll and pitch aren't unpacked in the API
  final StatusSignal<AngularVelocity> ss_roll_rate;
  final StatusSignal<AngularVelocity> ss_pitch_rate;
  final StatusSignal<AngularVelocity> ss_yaw_rate;
  final StatusSignal<Angle> ss_roll;
  final StatusSignal<Angle> ss_pitch;

  final StatusSignal<LinearAcceleration> ss_x_acc;
  final StatusSignal<LinearAcceleration> ss_y_acc;
  final StatusSignal<LinearAcceleration> ss_z_acc;

  // DPL - we can't construct CANcoders here, they will be constructed as part of
  // the swerve sub-system
  // CANCoders - monitor dt angles
  // CANcoder rot_encoder_bl = init(new CANcoder(CAN.DT_BL_CANCODER));
  // CANcoder rot_encoder_br = init(new CANcoder(CAN.DT_BR_CANCODER));
  // CANcoder rot_encoder_fl = init(new CANcoder(CAN.DT_FL_CANCODER));
  // CANcoder rot_encoder_fr = init(new CANcoder(CAN.DT_FR_CANCODER));

  // CAN monitoring
  final CANStatus m_canStatus;

  // Simulation
  /// AHRS_GyroSim m_gyroSim;

  // measured values
  double m_roll;
  double m_pitch;
  double m_yaw; // continous
  double m_yaw_mod180; // +-180
  double m_roll_d;
  double m_pitch_d;
  double m_yaw_d;

  // accelerations
  double m_ax;
  double m_ay;
  double m_az;

  double log_counter = 0;

  public Pose2d autoStartPose;
  public Pose2d autoEndPose;

  public Sensors_Subsystem() {

    // alocate sensors
    m_canStatus = new CANStatus();
    m_pigeon = new Pigeon2(CAN.PIGEON_IMU_CAN, "rio");
    // StatusSignal object
    ss_roll = m_pigeon.getRoll();
    ss_pitch = m_pigeon.getPitch();
    // not sure if these should be device or world
    ss_roll_rate = m_pigeon.getAngularVelocityXDevice();
    ss_pitch_rate = m_pigeon.getAngularVelocityYDevice();
    ss_yaw_rate = m_pigeon.getAngularVelocityZDevice();

    ss_x_acc = m_pigeon.getAccelerationX();
    ss_y_acc = m_pigeon.getAccelerationY();
    ss_z_acc = m_pigeon.getAccelerationZ();

    // setup network table
    table = NetworkTableInstance.getDefault().getTable("Sensors");
    positionTable = NetworkTableInstance.getDefault().getTable(NTStrings.NT_Name_Position);
    // nt_accelX = table.getEntry("x_dd");
    // nt_accelY = table.getEntry("y_dd");
    // nt_accelZ = table.getEntry("z_dd");

    nt_yaw_mod180 = table.getEntry("yaw_mod180");

    nt_canUtilization = table.getEntry("CanUtilization/value");
    nt_canRxError = table.getEntry("CanRxError");
    nt_canTxError = table.getEntry("CanTxError");

    // position angle encoders
    nt_yaw = table.getEntry("Active Yaw");
    nt_rotation = table.getEntry("Rotation");
    nt_pitch = positionTable.getEntry("Pitch");
    nt_roll = positionTable.getEntry("Roll");

    // allow Navx to calibrate in it's own thread
    new Thread(() -> {
      try {
        Thread.sleep(1000); // wait a second so gyro is done booting before calibrating.
        calibrate();
      } catch (Exception e) {
      }
    }).start();

    log(20);
  }

  // @Override
  public void calibrate() {
    reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_roll = ss_roll_rate.refresh().getValueAsDouble();
    m_pitch = ss_pitch_rate.refresh().getValueAsDouble();

    //double m_yaw = m_pigeon.getAngle(); //CCW+ signal
    m_yaw = m_pigeon.getYaw().getValueAsDouble();  // CW+}
    m_yaw_mod180 = ModMath.fmod360_2(m_yaw);

    // rates
    m_roll_d = ss_roll_rate.refresh().getValueAsDouble();
    m_pitch_d = ss_pitch_rate.refresh().getValueAsDouble();
    m_yaw_d = ss_yaw_rate.refresh().getValueAsDouble();

    // accelerations
    m_ax = ss_x_acc.refresh().getValueAsDouble() * g_mps2; // [m/s^2]
    m_ay = ss_y_acc.refresh().getValueAsDouble() * g_mps2; // [m/s^2]
    m_az = ss_z_acc.refresh().getValueAsDouble() * g_mps2; // [m/s^2]

    log(20);
  }

  // void updateYaw() {
  // m_pigeon.getAngle();
  // pigeon yaw is not modulated so needs modmath to get -180 to 180
  // m_yaw_pigeon = ModMath.fmod360_2(-m_pigeon.getYaw()); // CCW positive,
  // inverting here to match all the NavX code
  // previously written.
  // }

  void setupSimulation() {
    // m_gyroSim_ahrs = new AHRS_GyroSim(m_ahrs);
    // m_gyroSim SimDevice
  }

  @Override
  public void simulationPeriodic() {
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public void log(double mod) {

    log_counter++;
    if ((log_counter % mod) == 0) {
      // nt_accelX.setDouble(m_ahrs.getWorldLinearAccelX());
      // nt_accelY.setDouble(m_ahrs.getWorldLinearAccelY());
      // nt_accelZ.setDouble(m_ahrs.getWorldLinearAccelZ());

      nt_yaw_mod180.setDouble(m_yaw_mod180);

      CANJNI.getCANStatus(m_canStatus);
      nt_canUtilization.setDouble(m_canStatus.percentBusUtilization);
      nt_canRxError.setNumber(m_canStatus.receiveErrorCount);
      nt_canTxError.setNumber(m_canStatus.transmitErrorCount);

      nt_yaw.setDouble(getYaw());
      nt_rotation.setDouble(getRotation2d().getDegrees());
      nt_roll.setDouble(getRoll());
      nt_pitch.setDouble(getPitch());
    }
  }

  public void reset() {
    m_pigeon.reset();
  }

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

  public double getYawRate() {
    return m_yaw_d;
  }

  // @Override
  public void close() throws Exception {
    // //m_gyro.close();
    // m_gyro_ahrs.close();
  }

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

  public void setYaw(Rotation2d rot) {
    m_pigeon.setYaw(rot.getDegrees());
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
   * @return the current heading of the robot as a {@link
   *         edu.wpi.first.math.geometry.Rotation2d}.
   */
  // @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_yaw);
  }

  public double getXAcceleration() {
    return m_ax;
  }

  public double getYAcceleration() {
    return m_ay;
  }

  public double getZAcceleration() {
    return m_az;
  }

  public void setAutoStartPose(Pose2d pose) {
    autoStartPose = new Pose2d(pose.getTranslation(), pose.getRotation());
    setYaw(pose.getRotation()); // set gyro to starting heading so it's in field coordinates.
    System.out.println("***Auto Start Pose set: " + pose);
  }

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
    System.out.println("***Corrected End Pose: " + pose);

  }

}
