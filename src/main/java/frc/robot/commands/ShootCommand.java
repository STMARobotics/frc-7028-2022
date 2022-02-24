package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that finds a target, spins up the shooter, aims, and shoots at a the correct speed for the target distance.
 */
public class ShootCommand extends CommandBase {
  
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final PIDController pidController = new PIDController(AimConstants.kP, 0, AimConstants.kD);

  private double lastTargetDistance = 0;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      DriveTrainSubsystem driveTrainSubsystem,
      IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(shooterSubsystem, limelightSubsystem, driveTrainSubsystem, indexerSubsystem);

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
      double rotationSpeed = -pidController.calculate(limelightSubsystem.getTargetX());
      driveTrainSubsystem.arcadeDrive(0.0, rotationSpeed, false);
      if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
        indexerSubsystem.shoot();
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
  }

}
