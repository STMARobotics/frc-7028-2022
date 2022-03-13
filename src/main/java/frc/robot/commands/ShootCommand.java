package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that finds a target, spins up the shooter, aims, and shoots at a the correct speed for the target distance.
 */
public class ShootCommand extends CommandBase {
  
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private double lastTargetDistance = 0;
  private double lastTurretPosition = 0;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsytem,
      IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsytem;
    this.indexerSubsystem = indexerSubsystem;

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
      var atTarget = Math.abs(limelightSubsystem.getTargetX()) < AimConstants.AIM_TOLERANCE;
      if (shooterSubsystem.isReadyToShoot() && atTarget) {
        // Turn the indexer on to put cargo in shooter. It does not have safety so it will stay on until stopped.
        indexerSubsystem.shoot();
      } else {
        // Indexer can raise cargo up to the shooter while it's spinning up and aiming
        indexerSubsystem.prepareToShoot();
      }

      // Move the turret if it is outside of the aiming tolerance, otherwise hold the current position to prevent jitter
      if (!atTarget) {
        lastTurretPosition = turretSubsystem.getAngleToRobot() - limelightSubsystem.getTargetX();
      }
      turretSubsystem.positionToRobotAngle(lastTurretPosition);
    } else {
      // No target has ever been visible, so stop
      shooterSubsystem.stop();
      turretSubsystem.stop();
    }    
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
