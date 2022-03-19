package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to hold the turret at 180 degrees so it does not conflict with climb
 */
public class ClimbTurretSafetyCommand extends CommandBase {
  
  private final TurretSubsystem turretSubsystem;

  public ClimbTurretSafetyCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.positionToRobotAngle(180);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

}
