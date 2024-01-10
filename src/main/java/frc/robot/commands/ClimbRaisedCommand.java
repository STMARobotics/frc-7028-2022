package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to hold the turret at 180 degrees so it does not conflict with climb, and put the Limelight into driver mode
 */
public class ClimbRaisedCommand extends Command {
  
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ClimbRaisedCommand(
        TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(turretSubsystem, limelightSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.stop();
    limelightSubsystem.driverMode();
  }

  @Override
  public void execute() {
    shooterSubsystem.stop();
    turretSubsystem.positionToRobotAngle(180);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    limelightSubsystem.enable();
  }

}
