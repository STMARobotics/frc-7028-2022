package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to constantly aim the turret at the target. Probably only useful in autonomous.
 */
public class AimTurretCommand extends CommandBase {
  
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final DoubleSupplier targetAngleSupplier;
  
  private double lastTurretPosition;

  public AimTurretCommand(
        TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, DoubleSupplier targetAngleSupplier) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.targetAngleSupplier = targetAngleSupplier;

    addRequirements(turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
  }

  @Override
  public void execute() {
    if (limelightSubsystem.getTargetAcquired()) {
      // Target visible, position with limelight
      final var targetX = limelightSubsystem.getTargetX();
      final var atTarget = Math.abs(targetX) < AimConstants.AIM_TOLERANCE;
  
      if (!atTarget) {
        lastTurretPosition = turretSubsystem.getAngleToRobot() - targetX;
      }
      turretSubsystem.positionToRobotAngle(lastTurretPosition);
    } else {
      // Target not visible, position with odometry
      turretSubsystem.positionToRobotAngle(targetAngleSupplier.getAsDouble());
    }

  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

}
