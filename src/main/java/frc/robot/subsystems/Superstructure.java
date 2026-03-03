package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.HubShotSolver;
import frc.robot.util.LookUpTable;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ejml.sparse.csc.decomposition.lu.LuUpLooking_DSCC;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Superstructure extends SubsystemBase{

  public enum SuperstructureWantedState {
    IDLE,
    DRIVING,
    INTAKING,
    DASHBOARD,
    SHOOTING,
    REVUP,
    PRE_SHOOT

  }
  public enum SuperstructureCurrentState {
    IDLE, 
    DRIVING,
    INTAKING,
    DASHBOARD,
    SHOOTING,
    REVUP,
    PRE_SHOOT 
  }
    public Drive drive;
    public Shooter shooter;
    public Hood hood;
    public Intake intake;
    
    public SuperstructureWantedState wantedState = SuperstructureWantedState.DRIVING;
    public SuperstructureCurrentState currentState = SuperstructureCurrentState.DRIVING;
    private double targetDistanceMeters = 0.0;
    private static final double kShotIntervalSeconds = 0.2;
    private double lastShotTimestamp = 0.0;
    private static final double kShotExitHeightMeters = 0.6;
    private static final double kShotSolveMinAngleDeg = 10.0;
    private static final double kShotSolveMaxAngleDeg = 85.0;
    private static final double kShotSolveStepDeg = 0.1;
    private static final double kShotSolveMaxVelocityMps = 0.0;
    private static final double kAimToleranceDeg = 5;
    private static final double kShotArcDtSeconds = 0.02;
    private static final int kShotArcSamples = 75;
    private static final double kGravityMetersPerSecondSq = 9.81;
    private static final double kIntakeApproachKp = 1.5;
    private static final double kIntakeApproachMaxSpeedMps = 1.5;
    private static final double kIntakeStopDistanceMeters = 0.15;
           private HubShotSolver.Result target = new HubShotSolver.Result(0,0);
    private static final double kRpmToRps = 1.0 / 60.0;

    private final DoubleSubscriber shooterTargetRpmSub =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Dashboard/ShooterTargetRPM")
            .subscribe(0.0);
    private final BooleanSubscriber hoodInSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/HoodAngleDeg")
            .subscribe(false);
    private final DoubleSubscriber hoodSetpointDegSub =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Dashboard/HoodSetpointDeg")
            .subscribe(0.0);
    private final DoubleSubscriber shooterBoostRpmSub =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Dashboard/ShooterBoostRPM")
            .subscribe(0.0);
    private final BooleanSubscriber intakeInSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/IntakeIn")
            .subscribe(false);
    private final BooleanSubscriber spindexerEnabledSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Spindexer")
            .subscribe(false);
    private final BooleanSubscriber spindexerReverseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/SpindexerReverse")
            .subscribe(false);
    private double dashboardShooterRpm = 0.0;
    private double dashboardShooterBoostRpm = 0.0;
    private boolean dashboardIntakeIn = false;
    private boolean dashboardHoodIn = false;
    private double dashboardHoodSetpointDeg = 0.0;
    private boolean dashboardSpindexerEnabled = false;
    private boolean dashboardSpindexerReverse = false;
    private long lastDashboardSetpointChange = 0;

    private final SwerveRequest.RobotCentric intakeRequest =
        new SwerveRequest.RobotCentric().withDeadband(0.0).withRotationalDeadband(0.0);
    private final StructArrayPublisher<Pose3d> shotArcPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Shot/Arc", Pose3d.struct)
            .publish();

    @AutoLog
    public static class SuperstructureInputs {
    public String currentState = "IDLE";
    public String wantedState = "IDLE";
    public double targetDistanceMeters = 0.0;
    }



 

    @AutoLog
     public static class DriveTrainInputs {
    public Pose2d pose =  new Pose2d();
    public ChassisSpeeds speed = new ChassisSpeeds();
    public double xVelocity = 0;
    public double yVelocity = 0;
    public double thetaVelocity = 0;

  
    
  }

  private final DriveTrainInputsAutoLogged driveInputs = new DriveTrainInputsAutoLogged();
  private final SuperstructureInputsAutoLogged superstructureInputs = new SuperstructureInputsAutoLogged();


  public Superstructure(Drive drive, Shooter shooter, Hood hood, Intake intake){ {
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
    this.intake = intake;


  }
}

 @Override
    public void periodic() {
        Logger.recordOutput("RobotPose", drive.getPose());
        Logger.recordOutput("MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("AutoWinner", DriverStation.getGameSpecificMessage());
        Logger.recordOutput("Battery", RobotController.getBatteryVoltage());

        updateInputs();
        updateDashboardControls();
        


        Logger.processInputs("Superstructure", superstructureInputs);
        Logger.processInputs("DriveTrain", driveInputs);
        Logger.recordOutput("Aimed at the hub", isAimedAtHub());
        handleStates();
        applyStates();
        updateShotArcVisualization();

    
  }


    private void updateInputs() {
        driveInputs.xVelocity = drive.getChassisSpeeds().vxMetersPerSecond;
        driveInputs.yVelocity =  drive.getChassisSpeeds().vyMetersPerSecond;
        driveInputs.speed = drive.getChassisSpeeds();
        driveInputs.thetaVelocity = drive.getChassisSpeeds().omegaRadiansPerSecond;
        driveInputs.pose = drive.getPose();
        superstructureInputs.currentState = currentState.toString();
        superstructureInputs.wantedState = wantedState.toString();
         Translation2d hubTarget =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
                ? Constants.FieldConstants.HUB_RED.toTranslation2d()
                : Constants.FieldConstants.HUB_BLUE.toTranslation2d();
        superstructureInputs.targetDistanceMeters =
            drive.getPose().getTranslation().getDistance(hubTarget);

    }

    private void updateDashboardControls() {
        long shooterChange = shooterTargetRpmSub.getLastChange();
        long boostChange = shooterBoostRpmSub.getLastChange();
        long latestChange = Math.max(shooterChange, boostChange);
        if (latestChange > lastDashboardSetpointChange) {
            lastDashboardSetpointChange = latestChange;
            dashboardShooterRpm = shooterTargetRpmSub.get();
            dashboardShooterBoostRpm = shooterBoostRpmSub.get();
                wantedState = SuperstructureWantedState.DASHBOARD;
            
        }

        boolean hoodIn = hoodInSub.get();
        for (var sample : hoodInSub.readQueue()) {
            hoodIn = sample.value;
        }
        dashboardHoodIn = hoodIn;

        double hoodSetpointDeg = hoodSetpointDegSub.get();
        for (var sample : hoodSetpointDegSub.readQueue()) {
            hoodSetpointDeg = sample.value;
        }
        dashboardHoodSetpointDeg = hoodSetpointDeg;

        boolean intakeIn = intakeInSub.get();
        for (var sample : intakeInSub.readQueue()) {
            intakeIn = sample.value;
        }
        dashboardIntakeIn = intakeIn;

        boolean spindexerEnabled = spindexerEnabledSub.get();
        for (var sample : spindexerEnabledSub.readQueue()) {
            spindexerEnabled = sample.value;
        }
        dashboardSpindexerEnabled = spindexerEnabled;

        boolean spindexerReverse = spindexerReverseSub.get();
        for (var sample : spindexerReverseSub.readQueue()) {
            spindexerReverse = sample.value;
        }
        dashboardSpindexerReverse = spindexerReverse;
    }

    private void handleStates() {
        switch (wantedState) {
            case IDLE:
            
                currentState = SuperstructureCurrentState.IDLE;
                
                break;
            case DRIVING:
            // if(drive.getPose().getX() < 5.7){
            //     currentState = SuperstructureCurrentState.REVUP;
            // }
            // else{                
            //     currentState = SuperstructureCurrentState.DRIVING;
            //     }
                break;
            case INTAKING:
                currentState = SuperstructureCurrentState.INTAKING;
                break;
            case DASHBOARD:
                currentState = SuperstructureCurrentState.DASHBOARD;
                break;
            case SHOOTING:
                if(shootCheck()){               
                    
                     currentState = SuperstructureCurrentState.SHOOTING;
                    }
                else {
                    currentState = SuperstructureCurrentState.PRE_SHOOT;
                }
                break;
            case PRE_SHOOT:
            currentState = SuperstructureCurrentState.PRE_SHOOT;
        }
    }
    public boolean shootCheck(){
        HubShotSolver.Result target = target();
        this.target = target;
        if( Math.abs(shooter.rps() - shooter.targetShooterVelocity) < 30 && hood.isAtTargetAngle(hood.getTargetAngleDegrees()) ){
            return true;

        }
        else{
            return false;
        }
        
    }

    private void applyStates() {
        switch (currentState) {
            case IDLE:
            
            case DRIVING:
                shooter.setOff();
                if (dashboardHoodIn) {
                    hood.setIdle();
                }
                if (dashboardIntakeIn) {
                    intake.goHome();
                }
                shooter.setSpindexerManualEnabled(false);

                
                break;
            case INTAKING:
                shooter.setOff();
                hood.setIdle();
                intake.startIntake();
                shooter.setSpindexerManualEnabled(false);
                break;
            case DASHBOARD:
                shooter.dashboard(
                    (dashboardShooterRpm + dashboardShooterBoostRpm) * kRpmToRps,
                    dashboardSpindexerEnabled,
                    dashboardSpindexerReverse);
                if (dashboardHoodIn) {
                    hood.setIdle();
                } else {
                    hood.moveToAngle(dashboardHoodSetpointDeg);
                }
                intake.startIntake();
                break;
            case SHOOTING:
                shootingPipeline();
                // hood.moveToAngle(3);
                if (dashboardSpindexerReverse) {
                    shooter.setSpindexerManualOverride(true, true);
                } else {
                    shooter.setSpindexerManualEnabled(false);
                }
               
                break;

            case REVUP:
            shooter.preShoot(5,false);
            hood.setIdle();
            intake.goHome();
            shooter.setSpindexerManualEnabled(false);
            break;

            case PRE_SHOOT:
                 double disntance = 0;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            disntance = drive.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_RED.toTranslation2d());

        }
        else{
             disntance = drive.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_RED.toTranslation2d());


        }
            
               LookUpTable.LookUpTableTest lookUpTableTest = new LookUpTable().LookUpTableOutput(disntance);
            double shooterFinalRPM = lookUpTableTest.getRPS();
            double hooddeg = lookUpTableTest.getAngle();
            // shooter.preShoot(target.velocityMps);
            hood.moveToAngle(hooddeg );
double boostedShooterVelocity =
           ( shooterFinalRPM/60 ) + (dashboardShooterBoostRpm * kRpmToRps);
           if (drive.getChassisSpeeds().omegaRadiansPerSecond < 2.5) {
             shooter.preShoot(boostedShooterVelocity,true);       
            
           }
           else{
            shooter.preShoot(boostedShooterVelocity, false);
           }
            
            if (dashboardIntakeIn) {
                intake.goHome();
            }
            if (dashboardSpindexerReverse) {
                shooter.setSpindexerManualOverride(true, true);
            } else {
                shooter.setSpindexerManualEnabled(false);
            }
             break;
        }


    }

    public HubShotSolver.Result target(){
        double disntance = drive.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_BLUE.toTranslation2d());
        double targetHeightMeters = Constants.FieldConstants.HUB_BLUE.getZ();
        double robotSpeedTowardHub = getRobotSpeedTowardHubMetersPerSecond();
        HubShotSolver.Result targets =
            HubShotSolver.bestShotWithRobotSpeed(
                disntance,
                targetHeightMeters,
                kShotExitHeightMeters,
                robotSpeedTowardHub,
                kShotSolveMinAngleDeg,
                kShotSolveMaxAngleDeg,
                kShotSolveStepDeg,
                kShotSolveMaxVelocityMps);
        return targets;
    }

    public void shootingPipeline(){

        double disntance = 0;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            disntance = drive.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_RED.toTranslation2d());

        }
        else{
             disntance = drive.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_RED.toTranslation2d());


        }
        double targetHeightMeters = Constants.FieldConstants.HUB_BLUE.getZ();
        double robotSpeedTowardHub = getRobotSpeedTowardHubMetersPerSecond();
        HubShotSolver.Result targets = null;
         LookUpTable.LookUpTableTest lookUpTableTest = null;
        double shooterFinalRPM = 0;
        double hooddeg = 0;
        if(RobotBase.isSimulation()){
             targets =
            HubShotSolver.bestShotWithRobotSpeed(
                disntance,
                targetHeightMeters,
                kShotExitHeightMeters,
                robotSpeedTowardHub,
                kShotSolveMinAngleDeg,
                kShotSolveMaxAngleDeg,
                kShotSolveStepDeg,
                kShotSolveMaxVelocityMps);
        if (targets == null) {
            hood.setIdle();
            shooter.setOff();
            return;
        }
         double shooterAngularVelocityRadPerSec =
            targets.velocityMps / Units.inchesToMeters(2);

        //RPS
         shooterFinalRPM = shooterAngularVelocityRadPerSec/(2*Math.PI);
         hooddeg = targets.angleDeg;

        }

        else{

               lookUpTableTest = new LookUpTable().LookUpTableOutput(disntance);
            shooterFinalRPM = lookUpTableTest.getRPS();
            hooddeg = lookUpTableTest.getAngle();
        }

        
                   
        //Radian per sec
       

        hood.moveToAngle(hooddeg);
        double boostedShooterVelocity =
            (shooterFinalRPM/60)  + (dashboardShooterBoostRpm * kRpmToRps);
            Logger.recordOutput("boosted", boostedShooterVelocity);
        shooter.shoot(boostedShooterVelocity);

                if(shooter.isAtVelocity()&& isAimedAtHub() && drive.getChassisSpeeds().omegaRadiansPerSecond< Units.degreesToRadians(2)){
                    spawnFuelShot();
                }
            }
        
    


    public void requestIdle() {
        wantedState = SuperstructureWantedState.IDLE;
    }

    public void requestIntake() {
        wantedState = SuperstructureWantedState.INTAKING;
    }

    public void requestShooting() {
        wantedState = SuperstructureWantedState.SHOOTING;
    }
      public void requestShootingPRE() {
        wantedState = SuperstructureWantedState.PRE_SHOOT;
    }

    public void setTargetDistanceMeterps() {
        wantedState = SuperstructureWantedState.PRE_SHOOT;
    }

    public void setTargetDistanceMeters(double distanceMeters) {
        targetDistanceMeters = distanceMeters;
    }
    
    public Command setIntake(){
        return Commands.runOnce(() -> {
            requestIntake();
        });
    }
    public Command setIdle(){
        return Commands.runOnce(() -> {
            requestIdle();
        });

        }
    public Command setShooting(){
        return Commands.runOnce(() -> {
            requestShooting();
        });
    }
    public Command setPreshoot(){
        return Commands.runOnce(() -> {
            requestShootingPRE();
        });
    }
    public Command setDriving(){
        return Commands.runOnce(() -> {
            wantedState = SuperstructureWantedState.DRIVING;
        });    
    }

    public void spawnFuelShot() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        if (currentState != SuperstructureCurrentState.SHOOTING) {
            return;
        }
        if (!shooter.isAtVelocity()) {
            return;
        }
        if (!isAimedAtHub()) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (now - lastShotTimestamp < kShotIntervalSeconds) {
            return;
        }

        Pose2d robotPose = drive.getPose();
        Translation2d shooterOffset = new Translation2d(0.45, 0.0);
        ChassisSpeeds robotFieldSpeeds = drive.getChassisSpeeds();
        Rotation2d shooterFacing = robotPose.getRotation();
        double hoodAngleDegrees = hood.getTargetAngleDegrees();
        double shooterRPS = shooter.getTargetRPS(); //rps
        double shooterAngualrVelocity = shooterRPS * (2*Math.PI);
        

        double shooterSpeedMetersPerSecond =
            shooterAngualrVelocity * Units.inchesToMeters(2) ;

        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    shooterOffset,
                    robotFieldSpeeds,
                    shooterFacing,
                    Meters.of(kShotExitHeightMeters),
                    MetersPerSecond.of(shooterSpeedMetersPerSecond),
                    Degrees.of(hoodAngleDegrees)));
        lastShotTimestamp = now;
    }

    private void updateShotArcVisualization() {
        Pose2d robotPose = drive.getPose();
        Translation2d shooterOffset = new Translation2d(0.45, 0.0);
        Translation2d shooterFieldOffset = shooterOffset.rotateBy(robotPose.getRotation());
        Translation3d origin =
            new Translation3d(
                robotPose.getX() + shooterFieldOffset.getX(),
                robotPose.getY() + shooterFieldOffset.getY(),
                kShotExitHeightMeters);

        double distanceMeters =
            robotPose.getTranslation().getDistance(Constants.FieldConstants.HUB_BLUE.toTranslation2d());
        double targetHeightMeters = Constants.FieldConstants.HUB_BLUE.getZ();
        double robotSpeedTowardHub = getRobotSpeedTowardHubMetersPerSecond();
        HubShotSolver.Result solution =
            HubShotSolver.bestShotWithRobotSpeed(
                distanceMeters,
                targetHeightMeters,
                kShotExitHeightMeters,
                robotSpeedTowardHub,
                kShotSolveMinAngleDeg,
                kShotSolveMaxAngleDeg,
                kShotSolveStepDeg,
                kShotSolveMaxVelocityMps);
        if (solution == null) {
            shotArcPublisher.set(new Pose3d[0]);
            return;
        }

        double shooterSpeedMetersPerSecond = solution.velocityMps;
        double hoodAngleRad = Units.degreesToRadians(solution.angleDeg);
        double horizontalSpeed = shooterSpeedMetersPerSecond * Math.cos(hoodAngleRad);
        double verticalSpeed = shooterSpeedMetersPerSecond * Math.sin(hoodAngleRad);
        Translation2d hubDirection = getHubDirection();
        ChassisSpeeds robotFieldSpeeds = drive.getChassisSpeeds();
        double vx = robotFieldSpeeds.vxMetersPerSecond + horizontalSpeed * hubDirection.getX();
        double vy = robotFieldSpeeds.vyMetersPerSecond + horizontalSpeed * hubDirection.getY();

        List<Pose3d> samples = new ArrayList<>(kShotArcSamples);
        for (int i = 0; i < kShotArcSamples; i++) {
            double t = i * kShotArcDtSeconds;
            double x = origin.getX() + vx * t;
            double y = origin.getY() + vy * t;
            double z = origin.getZ() + verticalSpeed * t - 0.5 * kGravityMetersPerSecondSq * t * t;
            if (z < 0.0) {
                z = 0.0;
            }
            samples.add(new Pose3d(new Translation3d(x, y, z), new Rotation3d()));
            if (z <= 0.0 && i > 0) {
                break;
            }
        }

        shotArcPublisher.set(samples.toArray(new Pose3d[0]));
    }

    private Translation2d getHubDirection() {
        Pose2d robotPose = drive.getPose();
  Translation2d disntance = new Translation2d() ;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            disntance = Constants.FieldConstants.HUB_RED.toTranslation2d().minus(robotPose.getTranslation());

        }
        else{
            disntance = Constants.FieldConstants.HUB_BLUE.toTranslation2d().minus(robotPose.getTranslation());


        }        
     
        if (disntance.getNorm() < 1e-6) {
            return new Translation2d(1.0, 0.0);
        }
        return disntance.div(disntance.getNorm());
    }

    private double getRobotSpeedTowardHubMetersPerSecond() {
        Translation2d hubDirection = getHubDirection();
        ChassisSpeeds fieldSpeeds = drive.getChassisSpeeds();
        return fieldSpeeds.vxMetersPerSecond * hubDirection.getX()
            + fieldSpeeds.vyMetersPerSecond * hubDirection.getY();
    }

    public Rotation2d getHubHeading() {
        Translation2d hubDirection = getHubDirection();
        return new Rotation2d(hubDirection.getX(), hubDirection.getY()).plus(Rotation2d.fromDegrees(180));
    }

    public boolean isShootingRequested() {
        return wantedState == SuperstructureWantedState.SHOOTING;
    }

    // public Optional<SwerveRequest> getIntakeDriveRequest() {
    //     if (currentState != SuperstructureCurrentState.INTAKING) {
    //         return Optional.empty();
    //     }

    //     Optional<Translation2d> piece = vision.getBestPieceRobotRelative();
    //     if (piece.isEmpty()) {
    //         return Optional.empty();
    //     }

    //     Translation2d target = piece.get();
    //     if (target.getNorm() <= kIntakeStopDistanceMeters) {
    //         return Optional.of(intakeRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
    //     }

    //     double vx = MathUtil.clamp(target.getX() * kIntakeApproachKp, -kIntakeApproachMaxSpeedMps, kIntakeApproachMaxSpeedMps);
    //     double vy = MathUtil.clamp(target.getY() * kIntakeApproachKp, -kIntakeApproachMaxSpeedMps, kIntakeApproachMaxSpeedMps);

    //     return Optional.of(intakeRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(0.0));
    // }

    private boolean isAimedAtHub() {
        double desired = getHubHeading().getRadians();
        double current = drive.getPose().getRotation().getRadians();
        double error = MathUtil.angleModulus(desired - current);
        Logger.recordOutput("Hub", Math.abs(Units.radiansToDegrees(error)));
        return Math.abs(Units.radiansToDegrees(error)) <= kAimToleranceDeg;
    }

    }    

