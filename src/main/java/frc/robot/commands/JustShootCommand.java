package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to spin up the shooter and run the indexer to shoot cargo when the shooter is up to speed.
 * Used to manually get values for shooting interpolation table.
 */
public class JustShootCommand extends CommandBase {
  
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final DoubleSupplier speedSupplier;
  
  public JustShootCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, DoubleSupplier speedSupplier) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.speedSupplier = speedSupplier;

    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.runShooter(speedSupplier.getAsDouble());
    if (shooterSubsystem.isReadyToShoot()) {
      indexerSubsystem.shoot();
    } else {
      indexerSubsystem.prepareToShoot();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    indexerSubsystem.stop();
  }

}
