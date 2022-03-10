package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Command that finds a target, spins up the shooter, aims, and shoots at a the correct speed for the target distance.
 */
public class ShootCommand extends CommandBase {
  
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final TransferSubsystem transferSubsystem;

  private final PIDController pidController = new PIDController(AimConstants.kP, 0, AimConstants.kD);

  private double lastTargetDistance = 0;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      IndexerSubsystem indexerSubsystem, TransferSubsystem transferSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.transferSubsystem = transferSubsystem;

    addRequirements(shooterSubsystem, limelightSubsystem, driveTrainSubsystem, indexerSubsystem, transferSubsystem);

    pidController.setTolerance(AimConstants.AIM_TOLERANCE);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    lastTargetDistance = 0;
    pidController.reset();
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
      // Also note, the value passed to the PIDController is negated. The Limelight returns the position of the target
      // on a coordinate grid with the origin (X = 0) in the position where we want to put the cargo. To get the error
      // we subtract the current position from the origin. Since the origin is 0, we can just negate the current position.
      double rotationSpeed = pidController.calculate(-limelightSubsystem.getTargetX());
      if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
        // Turn the indexer on to put cargo in shooter. It does not have safety so it will stay on until stopped.
        indexerSubsystem.shoot();
        transferSubsystem.intake();
      } else {
        // Indexer can raise cargo up to the shooter while it's spinning and aiming
        indexerSubsystem.prepareToShoot();
      }
      if (pidController.atSetpoint()) {
        // Having a wide enough tolerance and stopping the drivetrain once at the setpoint prevents
        // jittering / oscalation when very close to the target
        driveTrainSubsystem.stop();
      } else {
        driveTrainSubsystem.arcadeDrive(0.0, rotationSpeed, false);
      }
    } else {
      // No target has ever been visible, so stop
      shooterSubsystem.stop();
      driveTrainSubsystem.stop();
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
    driveTrainSubsystem.stop();
    indexerSubsystem.stop();
    transferSubsystem.stop();
  }

}
