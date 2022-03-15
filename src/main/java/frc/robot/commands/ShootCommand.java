package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that finds a target, spins up the shooter, aims, and shoots at a the correct speed for the target distance.
 */
public class ShootCommand extends CommandBase {
  
  // The hub is in the center of the field. The field is 54' x 27'
  private static final Pose2d hubPose = 
      new Pose2d(Units.inchesToMeters(54 * 12) / 2, Units.inchesToMeters(27 * 12) / 2, new Rotation2d());

  private static final Pose2d fieldOriginOnHubPlane = 
      new Pose2d(-hubPose.getX(), -hubPose.getY(), new Rotation2d());

  private final ShooterSubsystem shooterSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final DoubleSupplier targetAngleProvider;
  private final boolean resetPose;
  private final int cargoToShoot;

  private int cargoShot = 0;
  private boolean wasFull = false;
  private Timer endTimer = new Timer();
  private double lastTargetDistance = 0;
  private double lastTurretPosition = 0;
  private boolean wrongColor = false;

  /**
   * Constructs a shoot command that will shoot at least the specified number of cargo
   * @param shooterSubsystem shooter subsystem
   * @param limelightSubsystem limelight subsystem
   * @param turretSubsytem turret subsystem
   * @param indexerSubsystem indexer subsystem
   * @param driveTrainSubsystem drivertrain subsystem
   * @param targetAngleProvider provider for predicted angle to the target (probably provided by odometry)
   * @param cargoToShoot number of cargo to shoot
   */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      DoubleSupplier targetAngleProvider,
      int cargoToShoot) {
    this(shooterSubsystem, limelightSubsystem, turretSubsytem, indexerSubsystem, driveTrainSubsystem,
        targetAngleProvider, false, cargoToShoot);
  }

  /**
   * Constructs a shoot command that will reset the robot's pose when the target is visible
   * @param shooterSubsystem shooter subsystem
   * @param limelightSubsystem limelight subsystem
   * @param turretSubsytem turret subsystem
   * @param indexerSubsystem indexer subsystem
   * @param driveTrainSubsystem drivertrain subsystem
   * @param targetAngleProvider provider for predicted angle to the target (probably provided by odometry)
   * @param resetPose true to reset the robot's pose when a target is found
   */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      DoubleSupplier targetAngleProvider,
      boolean resetPose) {
    this(shooterSubsystem, limelightSubsystem, turretSubsytem, indexerSubsystem, driveTrainSubsystem,
        targetAngleProvider, resetPose, Integer.MAX_VALUE);
  }

  /**
   * Constructs a shoot command that will shoot at least the specified number of cargo, and will
   * reset the robot's pose when the target is visible
   * @param shooterSubsystem shooter subsystem
   * @param limelightSubsystem limelight subsystem
   * @param turretSubsytem turret subsystem
   * @param indexerSubsystem indexer subsystem
   * @param driveTrainSubsystem drivertrain subsystem
   * @param targetAngleProvider provider for predicted angle to the target (probably provided by odometry)
   * @param resetPose true to reset the robot's pose when a target is found
   * @param cargoToShoot number of cargo to shoot
   */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      DoubleSupplier targetAngleProvider,
      boolean resetPose,
      int cargoToShoot) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsytem;
    this.indexerSubsystem = indexerSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.targetAngleProvider = targetAngleProvider;
    this.resetPose = resetPose;
    this.cargoToShoot = cargoToShoot;

    addRequirements(shooterSubsystem, limelightSubsystem, turretSubsytem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    lastTargetDistance = 0;
    lastTurretPosition = 0;
    cargoShot = 0;
    wasFull = indexerSubsystem.isFullSensorTripped();
    endTimer.reset();
  }

  @Override
  public void execute() {
    driveTrainSubsystem.stop();
    // If the target is visible, get the new distance. If the target isn't visible we'll use the last known distance.
    if (limelightSubsystem.getTargetAcquired()) {
      lastTargetDistance = limelightSubsystem.getDistanceToTarget();
    }

    // If we have a target distance, spin up and shoot
    if (lastTargetDistance > 0) {
      shooterSubsystem.prepareToShoot(lastTargetDistance);
      // We're not going to worry about losing the target for rotation because Limelight returns target X of 0 when no
      // target is visible, so we just won't rotate when no target is visible (although we may shoot since we're at
      // the setpoint)
      final var targetX = limelightSubsystem.getTargetX();
      final var atTarget = Math.abs(targetX) < AimConstants.AIM_TOLERANCE;
      if (shooterSubsystem.isReadyToShoot() && atTarget) {
        // Turn the indexer on to put cargo in shooter. It does not have safety so it will stay on until stopped.
        indexerSubsystem.shoot();
      } else {
        // Indexer can raise cargo up to the shooter while it's spinning up and aiming
        indexerSubsystem.prepareToShoot();
      }

      // Move the turret if it is outside of the aiming tolerance, otherwise hold the current position to prevent jitter
      if (!atTarget) {
        lastTurretPosition = turretSubsystem.getAngleToRobot() - targetX;
      }
      var alliance = DriverStation.getAlliance();
      switch(alliance) {
        case Blue:
          wrongColor = indexerSubsystem.getFullColor() != IndexerConstants.BLUE;
          break;
        case Red:
          wrongColor = indexerSubsystem.getFullColor() != IndexerConstants.RED;
          break;
        default:
          // don't change the wrong color value until a color is seen again
      }
      if (wrongColor) {
        // Point 3-degrees off to miss the shot
        // This might need to hold this position longer, it might snap back to position before the ball is shot
        turretSubsystem.positionToRobotAngle(lastTurretPosition - 3);
      } else {
        turretSubsystem.positionToRobotAngle(lastTurretPosition);
      }

      var isFull = indexerSubsystem.isFullSensorTripped();
      if ((wasFull && !isFull) && (++cargoShot >= cargoToShoot)) {
        System.out.println("Shot " + cargoShot + " cargo, waiting for timeout");
        endTimer.start();
      }
      wasFull = isFull;
      
      if (atTarget && resetPose) {
        // Reset robot pose
        final var gyroAngle = driveTrainSubsystem.getCurrentPose().getRotation().getDegrees();
        final var turretAngle = turretSubsystem.getAngleToRobot();
        final var distanceToTargetCenter = lastTargetDistance + AimConstants.HUB_TARGET_RADIUS;
        var newPose = new Pose2d(
          distanceToTargetCenter * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - targetX - 180)),
          distanceToTargetCenter * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - targetX - 180)),
          Rotation2d.fromDegrees(gyroAngle));
        driveTrainSubsystem.setCurrentPose(newPose.relativeTo(fieldOriginOnHubPlane));
      }
    } else {
      // No target has ever been visible, so point the turret where the target should be
      shooterSubsystem.stop();
      turretSubsystem.positionToRobotAngle(targetAngleProvider.getAsDouble());
    }
  }

  @Override
  public boolean isFinished() {
    return (cargoShot >= cargoToShoot && endTimer.hasElapsed(ShooterConstants.SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.disable();
    shooterSubsystem.stop();
    turretSubsystem.stop();
    indexerSubsystem.stop();
  }

}
