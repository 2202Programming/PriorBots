package frc.chadbot.bindings;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.PDPMonitorCmd;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.command.swerve.calibrate.TestConstantVelocity;
import frc.lib2202.command.swerve.calibrate.TestRotateVelocity;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2024.Constants.Tag_Pose;
import frc.robot2024.commands.Climber.Climb;
import frc.robot2024.commands.Etude.EtudeIntake;
import frc.robot2024.commands.Intake.AngleCalibration;
import frc.robot2024.commands.Intake.EjectNote;
import frc.robot2024.commands.Intake.InIntake;
import frc.robot2024.commands.Intake.IntakeSequence;
import frc.robot2024.commands.Intake.MoveToAnglePos;
import frc.robot2024.commands.Intake.TestIntakeAngle;
import frc.robot2024.commands.Shooter.CalibrateAngle;
import frc.robot2024.commands.Shooter.CalibrateWithLS;
import frc.robot2024.commands.Shooter.CalibrateZero;
import frc.robot2024.commands.Shooter.ShooterAngleSetPos;
import frc.robot2024.commands.Shooter.ShooterAngleVelMove;
import frc.robot2024.commands.Shooter.ShooterSequence;
import frc.robot2024.commands.Shooter.ShooterServoSequence;
import frc.robot2024.commands.Shooter.ShooterServoSequenceDebug;
import frc.robot2024.commands.Shooter.TestShoot;
import frc.robot2024.commands.auto.AutoShooting;
import frc.robot2024.commands.auto.AutoShooting.ShootingTarget;
//import frc.robot2024.subsystems.AmpMechanism;
import frc.robot2024.subsystems.Climber;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Shooter;
import frc.robot2024.subsystems.ShooterServo;

/*
 * Bindings here for testing, 
 */
public class Custom_ChadBot {

    // enum for bindings add when needed
    public enum Bindings {
        Competition,
        DriveTest, Shooter_test, IntakeTesting, auto_shooter_test, new_bot_test, comp_not_comp, Etude
    }

    static Bindings bindings = Bindings.DriveTest;

    public static void ConfigureOther(HID_Subsystem dc) { 
        DriverBinding(dc);
        OperatorBindings(dc);
    }

    // wrap the pathloading with try/catch
    static PathPlannerPath loadFromFile(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Big oops loading pn="+ pathName + ":" + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    static void DriverBinding(HID_Subsystem dc) {       
        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);        
        var intake = RobotContainer.getSubsystem(Intake.class);
        OdometryInterface odo = RobotContainer.getSubsystem("odometry");

        PathPlannerPath blue1 = loadFromFile("blue1");
        PathPlannerPath red1 = loadFromFile("red1");
        PathPlannerPath path_test_1m = loadFromFile("test_1m");
        
        CommandXboxController driver;

        if (dc.Driver() instanceof CommandXboxController) {
            driver = (CommandXboxController)dc.Driver();
        }
        else {
            DriverStation.reportError("Please use XBOX for Driver with this robot -- no driver bindings", false);
            return;  //no binding set
        }

        switch (bindings) {

            case DriveTest:
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.b().onTrue(new AllianceAwareGyroReset(false));

                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                if (blue1 != null)
                    driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(odo::printPose),
                        AutoBuilder.pathfindThenFollowPath(blue1,
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(odo::printPose)));
                if (red1 != null)
                    driver.b().onTrue(new SequentialCommandGroup(
                        new InstantCommand(odo::printPose),
                        AutoBuilder.pathfindThenFollowPath(red1,
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(odo::printPose)));

                // Start any watcher commands
                new PDPMonitorCmd(); // auto scheduled, runs when disabled
                
                driver.leftTrigger().onTrue(new ShooterSequence(true, 1200.0));
                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                if (path_test_1m != null)
                    driver.a().onTrue(new SequentialCommandGroup(
                        new InstantCommand(odo::printPose),
                        AutoBuilder.pathfindThenFollowPath(path_test_1m,
                                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(odo::printPose)));

                driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(odo::printPose),
                        AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
                                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(odo::printPose)));
                break;

            // i dont like that test commands and bindings are in here but we need them ig
            // --er
            case IntakeTesting:

                /*
                 * driver.rightBumper().whileTrue(new IntakeSequence(true));
                 * driver.povUp().onTrue(new ShooterSequence(true, 2000.0));
                 * driver.povRight().onTrue(new ShooterSequence(true, 1200.0));
                 * driver.povDown().whileTrue(new ShooterSequence(3200.0)); // RPM
                 * driver.leftBumper().whileTrue(new PneumaticsSequence());
                 * driver.x().whileTrue(new AngleCalibration(5.0));
                 * driver.y().whileTrue(new AngleCalibration(-5.0));
                 * // driver.leftBumper().whileTrue(new IntakeCalibrateForwardPos());
                 * driver.b().whileTrue(new TestIntake(0.35)); // % speed
                 * // driver.leftBumper().whileTrue(new TransferTest(30.0));
                 * driver.rightTrigger().onTrue(new MoveToAnglePos(Intake.TravelUp,
                 * Intake.TravelUp));
                 * driver.leftTrigger().onTrue(new MoveToAnglePos(Intake.TravelDown,
                 * Intake.TravelDown));
                 * // driver.rightTrigger().onTrue(new AnglePos(50.0));
                 * // driver.a().onTrue(new CalibratePos(0.0));
                 */
                driver.rightBumper().onTrue(new InstantCommand(() -> {
                    intake.setIntakeSpeed(0.0);
                }));
                driver.a().onTrue(new InstantCommand(() -> {
                    intake.setIntakeSpeed(2.0);
                }));
                driver.b().onTrue(new InstantCommand(() -> {
                    intake.setIntakeSpeed(-2.0);
                }));
                driver.x().onTrue(new InstantCommand(() -> {
                    intake.setAngleSetpoint(105.0);
                }));
                driver.y().onTrue(new InstantCommand(() -> {
                    intake.setAngleSetpoint(0.0);
                }));
                break;

            case auto_shooter_test:
                driver.y().onTrue(new AllianceAwareGyroReset(true));
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
                driver.povUp().onTrue(new CalibrateWithLS());
                driver.povUp().onTrue(new AngleCalibration(-25.0));
                driver.a().whileTrue(new IntakeSequence(false)
                        .andThen(new ShooterAngleSetPos(36.0)));
                driver.b().whileTrue(new EjectNote());
                driver.povDown().whileTrue(new ShooterAngleVelMove(-2));
                driver.povRight().whileTrue(new ShooterAngleVelMove(2));
                break;

            case Etude:
                driver.a().onTrue(new TestConstantVelocity(3.0, 6.0));
                driver.b().onTrue(new TestRotateVelocity(15.0, 6.0));
                driver.y().onTrue(new AllianceAwareGyroReset(true));
                driver.leftTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));

                break;

            case Shooter_test:
            case new_bot_test:
                driver.a().onTrue(new TestConstantVelocity(1.0, 4.0));
                driver.b().onTrue(new TestRotateVelocity(15.0, 6.0));
                driver.y().onTrue(new AllianceAwareGyroReset(true));
                driver.leftTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));

                break;
            case comp_not_comp:

                // Driver buttons
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.y().onTrue(new AllianceAwareGyroReset(true));
                driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
                break;

            default:
                break;
        }
    }


    static void OperatorBindings(HID_Subsystem dc) {        
        boolean skip_SS_only = false;
        CommandXboxController operator;
        // deal with xbox or joystick, these binding are xbox only
        if (dc.Driver() instanceof CommandXboxController) {
            operator = (CommandXboxController)dc.Operator();
        }
        else {
            DriverStation.reportError("Please use XBOX for Operator with this robot -- no opr bindings", false);
            return;  //no binding set
        }

        final Shooter shooter = (RobotContainer.getRobotName() == "CompetitionBotBeta2024")
                ? RobotContainer.getSubsystemOrNull(ShooterServo.class)
                : RobotContainer.getSubsystem(Shooter.class);
        if (!(shooter instanceof ShooterServo)) {
            System.out.println("ShooterServo not found, using AlphaShooter, some NPE happen depending on bindings.");
            skip_SS_only = true;
        }

        switch (bindings) {
            case Competition:
                Comp_ChadBot.OperatorBindings(dc);
                break;

            case auto_shooter_test:
                operator.rightTrigger().whileTrue(new ShooterAngleVelMove(4.0));
                operator.leftBumper().whileTrue(new ShooterAngleVelMove(-4.0));
                operator.a().onTrue(new IntakeSequence(false));
                operator.x().onTrue(new AngleCalibration(-25.0));
                operator.b().onTrue(new AutoShooting(ShootingTarget.Speaker));
                operator.y().whileTrue(new EjectNote());
                operator.povUp().onTrue(new ShooterAngleSetPos(45));
                operator.rightBumper().onTrue(new ShooterAngleSetPos(28.6));
                operator.povDown().onTrue(new ShooterAngleSetPos(30));
                operator.povLeft().onTrue(new CalibrateZero());
                break;

            case Shooter_test:
                operator.a().whileTrue(new IntakeSequence(false)
                        .andThen(new ShooterAngleSetPos(36.0)));
                operator.b().whileTrue(new EjectNote()); // eject note from intake
                operator.x().whileTrue(new InIntake(false)); // works ---> seq for stay in intake for amp shoot
                if (!skip_SS_only) {
                    operator.povUp().onTrue(new AngleCalibration(-25.0));// intake calibrate
                    operator.leftTrigger().onTrue(new ShooterServoSequenceDebug());
                    operator.rightTrigger().onTrue(
                            new ShooterServoSequence()); // auto shoot
                    // Shooter calibrate
                    operator.rightBumper().onTrue(new CalibrateWithLS());
                    operator.leftBumper().onTrue(new ShooterAngleSetPos(28.5)
                            .andThen(new WaitCommand(15.0)).andThen(new ShooterAngleSetPos(30.0))
                            .andThen(new WaitCommand(15.0)).andThen(new ShooterAngleSetPos(35.0))
                            .andThen(new WaitCommand(15.0)).andThen(new ShooterAngleSetPos(40.0))
                            .andThen(new WaitCommand(15.0)).andThen(new ShooterAngleSetPos(45.0))
                            .andThen(new WaitCommand(15.0)).andThen(new ShooterAngleSetPos(48.0)));
                    operator.povDown().whileTrue(new TestIntakeAngle(-20.0));
                    operator.povLeft().whileTrue(new TestIntakeAngle(20.0));
                    operator.povRight().onTrue(new ShooterAngleSetPos(ShooterServo.MAX_DEGREES));
                }

                break;

            case IntakeTesting:
                break;

            case new_bot_test:
                // // INTAKE & TRANSFER
                operator.x().whileTrue(new TestShoot(3000.0));
                operator.a().onTrue(new AngleCalibration(-10.0));
                if (!skip_SS_only) {
                    operator.rightTrigger().onTrue(new InstantCommand(() -> {
                        shooter.setExtensionPosition(0.0);
                    }));
                    operator.povUp().whileTrue(new ShooterAngleVelMove(1.0));
                    operator.povDown().whileTrue(new ShooterAngleVelMove(-1.0));

                    operator.povLeft().onTrue(new CalibrateAngle(-1.0));
                    operator.povRight().onTrue(new CalibrateAngle(1.0));
                    operator.rightBumper().onTrue(new CalibrateWithLS());
                }
                operator.y().whileTrue(new IntakeSequence(true));
                operator.b().whileTrue(new IntakeSequence(false));

                operator.leftBumper().onTrue(new InstantCommand(() -> {
                    shooter.setAngleSetpoint(28.52);
                }));
                operator.leftTrigger().onTrue(new InstantCommand(() -> {
                    shooter.setAngleSetpoint(36.0);
                }));

                break;
            case Etude:
                operator.a().whileTrue(new EtudeIntake(false));
                operator.b().whileTrue(new EjectNote());

                operator.povUp().whileTrue(new AngleCalibration(8.0));
                operator.rightBumper().onTrue(new ShooterSequence(true, 2000.0));
                operator.leftBumper().onTrue(new ShooterSequence(true, 800.0));
                operator.rightTrigger().onTrue(new ShooterSequence(3500.0));
                break;
            case comp_not_comp:
                var sideboard = dc.SwitchBoard();              
                // Switchboard buttons too
                sideboard.sw21().onTrue(new Climb(Climber.ExtendPosition));
                sideboard.sw22().onTrue(new Climb(Climber.ClimbPosition));
                sideboard.sw23().onTrue(new MoveToAnglePos(Intake.DownPos, Intake.TravelUp));

                /***************************************************************************************/
                // REAL COMPETITION BINDINGS.
                operator.a().whileTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            shooter.setAngleSetpoint(32.0);
                        }),
                        new IntakeSequence(false)));

                operator.b().whileTrue(new EjectNote()); // eject note from intake
                operator.x().whileTrue(new InIntake(false)); // works ---> seq for stay in intake for amp shoot
                operator.povUp().onTrue(new AngleCalibration(-25.0));// intake calibrate

                operator.rightBumper().onTrue(new ShooterServoSequence(46.5, 2200.0));
                operator.rightTrigger().onTrue(new ShooterServoSequence()); // was 35
                operator.leftTrigger().onTrue(new ShooterServoSequenceDebug());
                // Calibration commands
                operator.povUp().onTrue(new CalibrateWithLS());

            default:
                break;
        }
    }

}
