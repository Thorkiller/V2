// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Random;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.TunableController;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
public class RobotContainer {
    private SwerveDriveSimulation driveSimulation = null;
    private final Drive driveSub;
    private final Vision visionpose;

    private static final int kShooterCameraIndex = 0;
    private static final int kLeftCameraIndex = 1;
    private static final int kRightCameraIndex = 2;
    private static final String kShooterCameraName = "shooter";
    private static final String kLeftCameraName = "left1";
    private static final String kRightCameraName = "right1";

    // robotToCam: translation (x forward, y left, z up) and rotation (roll, pitch, yaw)
    private final Transform3d robotToShooterCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(0.18), Units.inchesToMeters(17)),
            new Rotation3d(0.0, Math.toRadians(-30), Math.toRadians(0)));

    private final Transform3d robotToLeftCam = new Transform3d(new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(0), Units.inchesToMeters(20)),
    new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(38)));

    private final Transform3d robotToRightCam = new Transform3d(new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(0), Units.inchesToMeters(20)), 
    new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(-39)));



    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Intake intake = new Intake();
    private final LoggedDashboardChooser<Command> autChooser;
    private final LoggedDashboardChooser<String> driveToPoseChooser;
    
    

    private final TunableController joystick = new TunableController(0);
          private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                    private double MaxAngularRatealighn = RotationsPerSecond.of(10).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static final double kDriveDeadband = 0.1;
    private static final double kIntakeAssistVisionDistanceMeters = 1.3;
    private static final int kIntakeAssistCameraIndex = 0;
    private static final double kIntakeAssistClusterYawWindowRad = Units.degreesToRadians(7.0);
    private final Superstructure superstructure;
    private final Random superstructureSwitchRng = new Random();


    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle aim = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
            .withMaxAbsRotationalRate(MaxAngularRatealighn)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(10, 0.0, 0.2);

  public RobotContainer() {

    switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                driveSub = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                superstructure = new Superstructure(driveSub, shooter, hood, intake);
                this.visionpose = new Vision(
                        driveSub,
                        this::shouldAcceptVisionPose,
                        new VisionIOPhotonVision(kShooterCameraName, robotToShooterCam),
                        new VisionIOPhotonVision(kLeftCameraName, robotToLeftCam),
                        new VisionIOPhotonVision(kRightCameraName, robotToRightCam));

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                driveSub = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                superstructure = new Superstructure(driveSub, shooter, hood, intake);
                visionpose = new Vision(
                        driveSub,
                        this::shouldAcceptVisionPose,
                        new VisionIOPhotonVisionSim(
                                kShooterCameraName, robotToShooterCam, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                kLeftCameraName, robotToLeftCam, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                kRightCameraName, robotToRightCam, driveSimulation::getSimulatedDriveTrainPose));

                break;

            default:
                // Replayed robot, disable IO implementations
                driveSub = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                superstructure = new Superstructure(driveSub, shooter, hood, intake);
                visionpose = new Vision(driveSub, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});

                break;
        }

        

    configureBindings();
    
    if( RobotBase.isSimulation()) {
      // In simulation, reset pose to origin on start
      driveSub.setPose(new Pose2d(3,3,new Rotation2d()));
     
    }
  NamedCommands.registerCommand(
    "shoot",
    Commands.sequence(
        superstructure.goIn(true),
        superstructure.setShooting(),
        DriveCommands.joystickDriveAtAngle(
                driveSub,
                () -> 0,
                () -> 0,
                superstructure::getHubHeading)
            .withTimeout(10).asProxy(),
        superstructure.goIn(false),
        superstructure.setDriving()));
  NamedCommands.registerCommand(
    "accshoot",
   superstructure.setShooting()
);

NamedCommands.registerCommand(
    "intake",
    superstructure.setIntake()
);
NamedCommands.registerCommand(
    "drive",
    Commands.sequence(superstructure.setDriving(),superstructure.goIn(true)));
NamedCommands.registerCommand(
    "faceforward",
    DriveCommands.joystickDriveAtAngle(
        driveSub,
        () -> 0,
        () -> 0,
        () -> Rotation2d.fromDegrees(0)).asProxy());


      autChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

      driveToPoseChooser = new LoggedDashboardChooser<>("Drive To Pose Target");
      AutoBuilder.getAllAutoNames().forEach(name -> driveToPoseChooser.addOption(name, name));
      autChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(driveSub));
      autChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(driveSub));
      autChooser.addOption(
          "Drive SysId (Quasistatic Forward)", driveSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autChooser.addOption(
          "Drive SysId (Quasistatic Reverse)", driveSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autChooser.addOption(
          "Drive SysId (Dynamic Forward)", driveSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autChooser.addOption(
          "Drive SysId (Dynamic Reverse)", driveSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  private void configureBindings() {

    

driveSub.setDefaultCommand(Commands.run(
                () -> driveSub.runVelocity(getAssistedTeleopSpeeds()),
                driveSub));

                Trigger rightTrigger = joystick.rightTrigger(0.199);
                Trigger leftTrigger = joystick.leftTrigger(0.199);
                Trigger leftBumper = joystick.leftBumper();
                Trigger rightBumper = joystick.rightBumper();
                Trigger aButton = joystick.a();
                Trigger bButton = joystick.b();
                Trigger xButton = joystick.x();
                Trigger yButton = joystick.y();
                Trigger startButton = joystick.start();

                bindControllerLogging("RightTrigger", rightTrigger);
                bindControllerLogging("LeftTrigger", leftTrigger);
                bindControllerLogging("LeftBumper", leftBumper);
                bindControllerLogging("RightBumper", rightBumper);
                bindControllerLogging("A", aButton);
                bindControllerLogging("B", bButton);
                bindControllerLogging("X", xButton);
                bindControllerLogging("Y", yButton);
                bindControllerLogging("Start", startButton);

               rightTrigger.whileTrue(
                   DriveCommands.joystickDriveAtAngle(
                       driveSub,
                       () -> -joystick.getLeftY(),
                       () -> -joystick.getLeftX(),
                       superstructure::getHubHeading,
                       superstructure::applyShootOnMoveSpeedLock));


            leftTrigger.onTrue(Commands.runOnce(() -> {
                superstructure.requestIntake();
              
            })).onFalse(superstructure.setDriving());
            rightTrigger
                
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> {
                     
                        superstructure.requestShooting();
                      
                    }))).onFalse(superstructure.setDriving());
                    
                    
                    leftBumper.onTrue(superstructure.goIn(true)).onFalse(superstructure.goIn(false));
                    rightBumper.onTrue(superstructure.spin(true)).onFalse(superstructure.spin(false));

    


                 aButton
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> {
                      if (shouldSwitchSuperstructure()) {
                        superstructure.requestShooting();
                      }
                    })));


                
            yButton.onTrue(Commands.runOnce(() -> {
              if (shouldSwitchSuperstructure()) {
                superstructure.wantedState = Superstructure.SuperstructureWantedState.DRIVING;
              }
            }));

            startButton
                .onTrue(superstructure.setReverseIntake())
                .onFalse(superstructure.setDriving());

            bButton.onTrue(superstructure.toggleShootOnMoveCommand());

            xButton.onTrue(Commands.runOnce(() -> {
                String autoName = driveToPoseChooser.get();
                if (autoName == null) return;
                try {
                    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                    if (paths.isEmpty()) return;
                    List<Pose2d> poses = paths.get(0).getPathPoses();
                    if (poses.isEmpty()) return;
                    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    Pose2d startPose = isRed ? FlippingUtil.flipFieldPose(poses.get(0)) : poses.get(0);
                    driveSub.setPose(startPose);
                } catch (Exception e) {}
            }));

            joystick.povUp().whileTrue(Commands.defer(this::buildDriveToPoseCommand, java.util.Set.of(driveSub)));


    
    
  }

  private void bindControllerLogging(String controlName, Trigger trigger) {
    trigger.onTrue(logControllerEvent(controlName, true));
    trigger.onFalse(logControllerEvent(controlName, false));
  }

  private Command logControllerEvent(String controlName, boolean pressed) {
    return Commands.runOnce(() -> {
      double timestampSeconds = Timer.getFPGATimestamp();
      String baseKey = "Controls/Driver/" + controlName;
      Logger.recordOutput(baseKey + "/Pressed", pressed);
      Logger.recordOutput(baseKey + "/EventTimestampSec", timestampSeconds);
      if (pressed) {
        Logger.recordOutput(baseKey + "/LastPressedTimestampSec", timestampSeconds);
      } else {
        Logger.recordOutput(baseKey + "/LastReleasedTimestampSec", timestampSeconds);
      }
    }).ignoringDisable(true);
  }

  private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDriveDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new edu.wpi.first.math.geometry.Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private ChassisSpeeds getAssistedTeleopSpeeds() {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(-joystick.getLeftY(), -joystick.getLeftX());
    double omega = MathUtil.applyDeadband(-joystick.getRightX(), kDriveDeadband);
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds driverFieldSpeeds = new ChassisSpeeds(
        linearVelocity.getX() * driveSub.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * driveSub.getMaxLinearSpeedMetersPerSec(),
        omega * driveSub.getMaxAngularSpeedRadPerSec());

    OptionalDouble clusterYawRadians = getBestClusterYawRadians(visionpose.getTargetObservations(kIntakeAssistCameraIndex));
    boolean hasVisionTarget = clusterYawRadians.isPresent();
    double visionLateralErrorMeters = hasVisionTarget
        ? Math.tan(clusterYawRadians.getAsDouble()) * kIntakeAssistVisionDistanceMeters
        : 0.0;

    ChassisSpeeds assistedFieldSpeeds = superstructure.applyIntakeAssist(
        driverFieldSpeeds,
        visionLateralErrorMeters,
        hasVisionTarget);
    assistedFieldSpeeds = superstructure.applyShootOnMoveSpeedLock(assistedFieldSpeeds);

    boolean isFlipped = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        assistedFieldSpeeds,
        isFlipped ? driveSub.getRotation().plus(new Rotation2d(Math.PI)) : driveSub.getRotation());
  }

  private OptionalDouble getBestClusterYawRadians(TargetObservation[] observations) {
    if (observations == null || observations.length == 0) {
      return OptionalDouble.empty();
    }

    double[] yaws = Arrays.stream(observations)
        .mapToDouble(observation -> observation.tx().getRadians())
        .sorted()
        .toArray();

    int bestStart = 0;
    int bestEnd = 0;
    int start = 0;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (int end = 0; end < yaws.length; end++) {
      while (start <= end && (yaws[end] - yaws[start]) > kIntakeAssistClusterYawWindowRad) {
        start++;
      }

      int count = end - start + 1;
      double sumYaw = 0.0;
      for (int i = start; i <= end; i++) {
        sumYaw += yaws[i];
      }
      double centerYaw = sumYaw / count;
      double score = (count * 100.0) - Math.abs(centerYaw);
      if (score > bestScore) {
        bestScore = score;
        bestStart = start;
        bestEnd = end;
      }
    }

    double bestSumYaw = 0.0;
    for (int i = bestStart; i <= bestEnd; i++) {
      bestSumYaw += yaws[i];
    }
    return OptionalDouble.of(bestSumYaw / (bestEnd - bestStart + 1));
  }

  private boolean shouldAcceptVisionPose(int cameraIndex) {
    boolean shooterCameraEnabled;

    if (DriverStation.isTeleopEnabled()) {
      shooterCameraEnabled = true;
    } else if (DriverStation.isDisabled()) {
      shooterCameraEnabled = true;
    } else if (DriverStation.isAutonomousEnabled()) {
      shooterCameraEnabled = superstructure != null
          && superstructure.shouldUseShooterCameraInAuto();
    } else {
      shooterCameraEnabled = false;
    }

    Logger.recordOutput("Vision/ShooterCameraEnabled", shooterCameraEnabled);

    if (cameraIndex != kShooterCameraIndex) {
      return DriverStation.isTeleopEnabled();
    }

    return shooterCameraEnabled;
  }

  private boolean shouldSwitchSuperstructure() {
    return superstructureSwitchRng.nextInt(100) < 81;
  }

private Command buildDriveToPoseCommand() {
    String autoName = driveToPoseChooser.get();
    if (autoName == null) return Commands.none();

    List<PathPlannerPath> paths;
    try {
        paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    } catch (Exception e) {
        return Commands.none();
    }
    if (paths.isEmpty()) return Commands.none();

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    List<Pose2d> allPoses = new ArrayList<>();
    for (PathPlannerPath path : paths) {
        List<Pose2d> pathPoses = path.getPathPoses();
        // subsample: every 10th pose + always include the last
        for (int i = 0; i < pathPoses.size(); i++) {
            if (i % 10 == 0 || i == pathPoses.size() - 1) {
                Pose2d p = isRed ? FlippingUtil.flipFieldPose(pathPoses.get(i)) : pathPoses.get(i);
                allPoses.add(p);
            }
        }
    }
    if (allPoses.isEmpty()) return Commands.none();

    driveSub.setPose(allPoses.get(0));

    List<Command> commands = new ArrayList<>();
    for (int i = 0; i < allPoses.size(); i++) {
        double tolerance = (i == allPoses.size() - 1) ? 0.05 : 0.25;
        commands.add(pidDriveToPose(allPoses.get(i), tolerance));
    }
    return Commands.sequence(commands.toArray(new Command[0]));
}

private Command pidDriveToPose(Pose2d target, double translationToleranceMeters) {
    var xPid = new edu.wpi.first.math.controller.PIDController(3.0, 0, 0.0);
    var yPid = new edu.wpi.first.math.controller.PIDController(3.0, 0, 0.0);
    var thetaPid = new edu.wpi.first.math.controller.ProfiledPIDController(
        2.0, 0.0, 0.1,
        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(6, 20));
    thetaPid.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(() -> {
        Pose2d current = driveSub.getPose();
        double xSpeed = xPid.calculate(current.getX(), target.getX());
        double ySpeed = yPid.calculate(current.getY(), target.getY());
        double omega = thetaPid.calculate(
            current.getRotation().getRadians(), target.getRotation().getRadians());
        driveSub.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, omega, current.getRotation()));
    }, driveSub)
    .until(() -> driveSub.getPose().getTranslation().getDistance(target.getTranslation()) < translationToleranceMeters)
    .finallyDo(() -> { xPid.close(); yPid.close(); });
}

public Command getAutonomousCommand() {
    // Command selectedAuto = autChooser.get();
    // if (selectedAuto instanceof PathPlannerAuto pathPlannerAuto
    //     && pathPlannerAuto.getStartingPose() != null) {
    //   return Commands.sequence(
    //       AutoBuilder.resetOdom(pathPlannerAuto.getStartingPose()),
    //       selectedAuto.asProxy());
    // }
    // return selectedAuto;

    return autChooser.get();



  }

  public void resetSimOnEnable() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    driveSub.setPose(new Pose2d(3, 3, new Rotation2d()));
  }

  private void spawnFuelCluster(int count) {
    if (!RobotBase.isSimulation()) {
      return;
    }

    Translation2d center = new Translation2d(8.27, 4.035);
    double spreadMeters = 0.6;

    for (int i = 0; i < count; i++) {
      double angle = Math.random() * Math.PI * 2.0;
      double radius = Math.sqrt(Math.random()) * spreadMeters;
      Translation2d offset = new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle));
      SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(center.plus(offset)));
    }
  }

  private void clearSimGamePieces() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    SimulatedArena.getInstance().clearGamePieces();
  }

  private void spawnFuelShot() {
    superstructure.spawnFuelShot();
  }

   public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().resetFieldForAuto();
    }

}




