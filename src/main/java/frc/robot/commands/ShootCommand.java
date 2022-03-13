package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
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
  private final boolean resetPose;
  private final int ballsToShoot;

  private double lastTargetDistance = 0;
  private double lastTurretPosition = 0;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      int ballsToShoot) {
    this(shooterSubsystem, limelightSubsystem, turretSubsytem, indexerSubsystem, driveTrainSubsystem, false, ballsToShoot);
  }

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      boolean resetPose) {
    this(shooterSubsystem, limelightSubsystem, turretSubsytem, indexerSubsystem, driveTrainSubsystem, resetPose, Integer.MAX_VALUE);
  }

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      boolean resetPose,
      int ballsToShoot) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsytem;
    this.indexerSubsystem = indexerSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.resetPose = resetPose;
    this.ballsToShoot = ballsToShoot;

    addRequirements(shooterSubsystem, limelightSubsystem, turretSubsytem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    lastTargetDistance = 0;
    lastTurretPosition = 0;
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
      turretSubsystem.positionToRobotAngle(lastTurretPosition);
      if (atTarget && resetPose) {
        // Reset robot pose
        final var gyroAngle = driveTrainSubsystem.getCurrentPose().getRotation().getDegrees();
        final var turretAngle = turretSubsystem.getAngleToRobot();
        var newPose = new Pose2d(
          lastTargetDistance * Math.cos(Units.degreesToRadians(gyroAngle + turretAngle - targetX - 180)),
          lastTargetDistance * Math.sin(Units.degreesToRadians(gyroAngle + turretAngle - targetX - 180)),
          Rotation2d.fromDegrees(gyroAngle));
        driveTrainSubsystem.setCurrentPose(newPose.relativeTo(fieldOriginOnHubPlane));
      }
    } else {
      // No target has ever been visible, so point the turret where the target should be
      shooterSubsystem.stop();
      pointTurretTowardTarget();
    }
  }

  /**
   * Uses the robot's pose on the field to turn the turret at the hub
   */
  private void pointTurretTowardTarget() {
    // For all measurements, the grid is on the field with the X axis running the long way across the field, with the
    // the blue alliance on the lesser X side, and the red alliance on the greater X side. The Y axis runs the shot
    // way across the field. CCW rotation is positive.

    // The robot's pose is on a coordinate grid with (0,0) at the bottom left corner.
    var robotCurrentPose = driveTrainSubsystem.getCurrentPose();

    // Get the robot's pose relative to the target. This will be the pose of the robot on a grid with the hub in the
    // center at (0,0)
    var relativePose = robotCurrentPose.relativeTo(hubPose);

    // Get the angle to the hub from the robot's position
    var angleToHub = Math.atan(relativePose.getY() / relativePose.getX());

    // Calculate the direction to turn the turret, considering the direction the robot's drivetrain is in
    double headingSetpoint = Units.radiansToDegrees(angleToHub) - robotCurrentPose.getRotation().getDegrees();

    // Deal with quadrants where X is > 0 (the robot is to the right of the hub)
    if (relativePose.getX() > 0) {
      headingSetpoint += 180;
    }

    // Deal with quadrants that result in a negative or out of range value
    headingSetpoint = (headingSetpoint + 360) % 360;

    // Position the turret
    turretSubsystem.positionToRobotAngle(headingSetpoint);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.disable();
    shooterSubsystem.stop();
    turretSubsystem.stop();
    indexerSubsystem.stop();
  }

}
