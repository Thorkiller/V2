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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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


public class RobotContainer {

  PhotonCamera leftCam = new PhotonCamera("LeftCam");
PhotonCamera rightCam = new PhotonCamera("RightCam");
    private SwerveDriveSimulation driveSimulation = null;
        private final Drive driveSub;
            private final Vision visionpose;



// robotToCam: translation (x forward, y left, z up) and rotation (roll, pitch, yaw)
Transform3d robotToLeftCam  = new Transform3d(
    new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(0.003), Units.inchesToMeters(17)),
    new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(180))
);

Transform3d robotToRightCam = new Transform3d(
    new Translation3d(0.25, -0.18, 0.45),
    new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(-10))
);



    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Intake intake = new Intake();
    private final LoggedDashboardChooser<Command> autChooser;
    
    

    private final TunableController joystick = new TunableController(0);
          private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                    private double MaxAngularRatealighn = RotationsPerSecond.of(10).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
                this.visionpose = new Vision(
                        driveSub,
                        new VisionIOPhotonVision("shooter", robotToLeftCam)
                       );

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
                visionpose = new Vision(
                        driveSub,
                        new VisionIOPhotonVisionSim(
                                "camera0Name", robotToLeftCam, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                "camera1Name", robotToLeftCam, driveSimulation::getSimulatedDriveTrainPose));

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
                visionpose = new Vision(driveSub, new VisionIO() {}, new VisionIO() {});

                break;
        }
        superstructure = new Superstructure(driveSub, shooter, hood, intake);

        

    configureBindings();
    
    if( RobotBase.isSimulation()) {
      // In simulation, reset pose to origin on start
      driveSub.setPose(new Pose2d(3,3,new Rotation2d()));
     
    }
  NamedCommands.registerCommand(
    "shoot",
    DriveCommands.joystickDriveAtAngle(driveSub, () -> 0, () -> 0, superstructure::getHubHeading)
        .alongWith(Commands.run(() -> superstructure.requestShootingPRE()))
);

      autChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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

    

driveSub.setDefaultCommand(DriveCommands.joystickDrive(
                driveSub, () -> -joystick.getLeftY(), () -> -joystick.getLeftX(), () -> -joystick.getRightX()));

                joystick.rightTrigger(0.2).whileTrue(DriveCommands.joystickDriveAtAngle(driveSub, ()-> 0, ()-> 0,()-> (superstructure.getHubHeading())));


            joystick.leftTrigger(0.2).onTrue(Commands.runOnce(() -> {
                superstructure.requestIntake();
              
            }));
            joystick.rightTrigger(0.2)
                
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> {
                     
                        superstructure.requestShootingPRE();
                      
                    }))).onFalse(superstructure.setDriving());
                    
                    
                    joystick.leftBumper().onTrue(superstructure.goIn(true)).onFalse(superstructure.goIn(false));
                    joystick.rightBumper().onTrue(superstructure.spin(true)).onFalse(superstructure.spin(false));

    


                 joystick.a()
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> {
                      if (shouldSwitchSuperstructure()) {
                        superstructure.requestShooting();
                      }
                    })));


                
            joystick.y().onTrue(Commands.runOnce(() -> {
              if (shouldSwitchSuperstructure()) {
                superstructure.wantedState = Superstructure.SuperstructureWantedState.DRIVING;
              }
            }));

            joystick.x().whileTrue(Commands.runOnce(()-> driveSub.setPose(new Pose2d(driveSub.getPose().getX(),driveSub.getPose().getY(),new Rotation2d())))); 


    
    
  }

  private boolean shouldSwitchSuperstructure() {
    return superstructureSwitchRng.nextInt(100) < 81;
  }

  public Command getAutonomousCommand() {
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
