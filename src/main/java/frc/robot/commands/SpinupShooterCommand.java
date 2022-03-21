package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinupShooterCommand extends CommandBase {
  
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

}
