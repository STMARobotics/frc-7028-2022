package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinupShooterCommand extends Command {
  
  private final ShooterSubsystem shooterSubsystem;
  private final double distance;

  public SpinupShooterCommand(ShooterSubsystem shooterSubsystem, double distance) {
    this.shooterSubsystem = shooterSubsystem;
    this.distance = distance;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.prepareToShoot(distance);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

}
