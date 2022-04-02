package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Positions the turret to an angle
 */
public class PositionTurretCommand extends CommandBase {
  
  private final TurretSubsystem turretSubsystem;
  private final double angle;

  public PositionTurretCommand(TurretSubsystem turretSubsystem, double angle) {
    this.turretSubsystem = turretSubsystem;
    this.angle = angle;

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.positionToRobotAngle(angle);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

}
