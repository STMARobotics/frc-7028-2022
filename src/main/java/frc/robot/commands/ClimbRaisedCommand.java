package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to hold the turret at 180 degrees so it does not conflict with climb, and put the Limelight into driver mode
 */
public class ClimbRaisedCommand extends CommandBase {
  
  private final TurretSubsystem turretSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public ClimbRaisedCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.driverMode();
  }

  @Override
  public void execute() {
    turretSubsystem.positionToRobotAngle(180);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    limelightSubsystem.enable();
  }

}
