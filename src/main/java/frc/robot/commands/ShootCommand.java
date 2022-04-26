package frc.robot.commands;

import static frc.robot.Constants.AimConstants.AIM_ROTATION_SPEED;
import static frc.robot.Constants.ShooterConstants.MAX_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MIN_DISTANCE;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that finds a target, spins up the shooter, aims, and shoots at a the correct speed for the target distance.
 */
public class ShootCommand extends CommandBase {
  
  private static final Pose2d hubPose = 
      new Pose2d(Units.inchesToMeters(54 * 12) / 2, Units.inchesToMeters(27 * 12) / 2, new Rotation2d());

  private static final Pose2d fieldOriginOnHubPlane = 
      new Pose2d(-hubPose.getX(), -hubPose.getY(), new Rotation2d());

  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(DriverConstants.ROTATE_RATE_LIMIT_ARCADE);

  private final ShooterSubsystem shooterSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final DoubleSupplier targetAngleProvider;
  private final boolean resetPose;
  private final int cargoToShoot;
  private final DoubleConsumer rumble;

  private int cargoShot = 0;
  private boolean wasFull = false;
  private double lastTargetDistance = 0;
  private double lastTurretPosition = 0;
  private boolean wrongColor = false;
  private double missTurretOffset = 0d;
  private boolean isShooting;

  /**
   * Constructs a shoot command that will shoot at least the specified number of cargo
   * @param shooterSubsystem shooter subsystem
   * @param limelightSubsystem limelight subsystem
   * @param turretSubsystem turret subsystem
   * @param indexerSubsystem indexer subsystem
   * @param driveTrainSubsystem drivertrain subsystem
   * @param targetAngleProvider provider for predicted angle to the target (probably provided by odometry)
   * @param cargoToShoot number of cargo to shoot
   */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsystem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      DoubleSupplier targetAngleProvider,
      int cargoToShoot) {
    this(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem, driveTrainSubsystem,
        targetAngleProvider, false, cargoToShoot, null);
  }

  /**
   * Constructs a shoot command that will reset the robot's pose when the target is visible
   * @param shooterSubsystem shooter subsystem
   * @param limelightSubsystem limelight subsystem
   * @param turretSubsystem turret subsystem
   * @param indexerSubsystem indexer subsystem
   * @param driveTrainSubsystem drivertrain subsystem
   * @param targetAngleProvider provider for predicted angle to the target (probably provided by odometry)
   * @param resetPose true to reset the robot's pose when a target is found
   */
  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsystem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      DoubleSupplier targetAngleProvider,
      boolean resetPose,
      DoubleConsumer rumble) {
    this(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem, driveTrainSubsystem,
        targetAngleProvider, resetPose, Integer.MAX_VALUE, rumble);
  }

  /**
   * Constructs a shoot command that will shoot at least the specified number of cargo, and will
   * reset the robot's pose when the target is visible
   * @param shooterSubsystem shooter subsystem
   * @param limelightSubsystem limelight subsystem
   * @param turretSubsystem turret subsystem
   * @param indexerSubsystem indexer subsystem
   * @param driveTrainSubsystem drivertrain subsystem
   * @param targetAngleProvider provider for predicted angle to the target (probably provided by odometry)
   * @param resetPose true to reset the robot's pose when a target is found
   * @param cargoToShoot number of cargo to shoot
   */
  private ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsystem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      DoubleSupplier targetAngleProvider,
      boolean resetPose,
      int cargoToShoot,
      DoubleConsumer rumble) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.targetAngleProvider = targetAngleProvider;
    this.resetPose = resetPose;
    this.cargoToShoot = cargoToShoot;
    this.rumble = rumble == null ? (r) -> {} : rumble;

    addRequirements(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem, driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    lastTargetDistance = 0;
    lastTurretPosition = turretSubsystem.getAngleToRobot();
    cargoShot = 0;
    wasFull = indexerSubsystem.isFullSensorTripped();
    wrongColor = false;
    missTurretOffset = 0;
    isShooting = false;
  }

  @Override
  public void execute() {
    // If the target is visible, get the new distance. If the target isn't visible we'll use the last known distance.
    if (limelightSubsystem.getTargetAcquired()) {
      lastTargetDistance = limelightSubsystem.getDistanceToTarget();
    }

    // If we have a target distance, spin up and shoot
    if (lastTargetDistance > 0) {
      if (lastTargetDistance > MAX_DISTANCE || lastTargetDistance < MIN_DISTANCE) {
        // Target out of range, rumble and exit
        rumble.accept(1);
        shooterSubsystem.prepareToShoot(MIN_DISTANCE);
        indexerSubsystem.stop();
        aimAtTarget(targetAngleProvider.getAsDouble());
        return;
      }
      rumble.accept(0);

      shooterSubsystem.prepareToShoot(lastTargetDistance);
      // We're not going to worry about losing the target for rotation because Limelight returns target X of 0 when no
      // target is visible, so we just won't rotate when no target is visible (although we may shoot since we're at
      // the setpoint)
      final var targetX = limelightSubsystem.getTargetX();
      final var atTarget = Math.abs(targetX) < AimConstants.AIM_TOLERANCE;

      // Update the wrongColor variable
      checkAllianceColor();

      if ((shooterSubsystem.isReadyToShoot() && (wrongColor || atTarget)) || isShooting) {
        // Turn the indexer on to put cargo in shooter
        indexerSubsystem.shoot();
        isShooting = true;
      } else {
        // Indexer can raise cargo up to the shooter while it's spinning up and aiming
        indexerSubsystem.prepareToShoot();
      }

      // Move the turret if it is outside of the aiming tolerance, otherwise hold the current position to prevent jitter
      if (!atTarget) {
        lastTurretPosition = turretSubsystem.getAngleToRobot() - targetX;
      }
      if (wrongColor) {
        if (missTurretOffset == 0) {
          // Aim off from the target to miss the shot for wrong color cargo
          // Only do this once to prevent bouncing back and forth near 180-degrees
          missTurretOffset = lastTurretPosition > 180 ? -20 : 20;
        }
        aimAtTarget(lastTurretPosition + missTurretOffset);
      } else {
        aimAtTarget(lastTurretPosition);
      }

      // Check for full sensor falling edge while shooting (a ball left the top of the indexer)
      var isFull = indexerSubsystem.isFullSensorTripped();
      if (wasFull && !isFull && isShooting) {
        cargoShot++;
        System.out.println("Shot " + cargoShot + " cargo");
        isShooting = false;
      }
      wasFull = isFull;
      
      if (atTarget && resetPose) {
        // Reset robot pose
        resetRobotPose(targetX);
      }
    } else {
      // No target has ever been visible, so aim where the target should be
      shooterSubsystem.prepareToShoot(MIN_DISTANCE);
      aimAtTarget(targetAngleProvider.getAsDouble());
    }
  }

  /**
   * Aims at the target by positioning the turret, and turning the drivetrain if the target it out of reach.
   */
  private void aimAtTarget(double angle) {
    if (TurretSubsystem.isInShootingRange(angle) || !resetPose) {
      driveTrainSubsystem.stop();
      rotationRateLimiter.reset(0);
    } else {
      // Turn the drivetrain toward the target
      var rotation = angle < 180 ? AIM_ROTATION_SPEED : -AIM_ROTATION_SPEED;
      driveTrainSubsystem.arcadeDrive(0, rotationRateLimiter.calculate(rotation), false);
    }
    turretSubsystem.positionToRobotAngle(angle);
  }

  /**
   * Checks of the color of the cargo matches the current alliance and sets the
   * wrongColor variable
   */
  private void checkAllianceColor() {
    // Change the value of wrongColor only if a color is visible to prevent snapping
    // back to the target between the time the cargo leaves the top sensor to when it
    // is shot by the shooter
    var alliance = DriverStation.getAlliance();
    var cargoColor = indexerSubsystem.getFullColor();
    if (cargoColor == IndexerConstants.COLOR_BLUE) {
      wrongColor = alliance != Alliance.Blue;
    } else if (cargoColor == IndexerConstants.COLOR_RED) {
      wrongColor = alliance != Alliance.Red;
    }
  }

  /**
   * Resets the robot pose from the target angle. Does trig with the distance to the target,
   * the current gyro angle, and the target X.
   * @param targetX target X from limelight
   */
  private void resetRobotPose(final double targetX) {
    final var gyroAngle = driveTrainSubsystem.getCurrentPose().getRotation().getDegrees();
    final var turretAngle = turretSubsystem.getAngleToRobot();
    final var distanceToTargetCenter = lastTargetDistance + AimConstants.HUB_TARGET_RADIUS;
    var newPose = new Pose2d(
      distanceToTargetCenter * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - targetX - 180)),
      distanceToTargetCenter * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - targetX - 180)),
      Rotation2d.fromDegrees(gyroAngle));
    driveTrainSubsystem.setCurrentPose(newPose.relativeTo(fieldOriginOnHubPlane));
  }

  @Override
  public boolean isFinished() {
    return (cargoShot >= cargoToShoot);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    indexerSubsystem.stop();
    driveTrainSubsystem.stop();
    rotationRateLimiter.reset(0);
    rumble.accept(0);
  }

}
