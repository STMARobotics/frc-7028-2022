package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Command to load cargo in autonomous that also runs the indexer.
 * This is needed because a command group inherits all of its requirements.
 * That means an autonomous routine that uses the indexer for shooting will
 * not run the indexer's default command.
 */
public class LoadCargoWithIndexerCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TransferSubsystem transferSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public LoadCargoWithIndexerCommand(
        IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IndexerSubsystem indexerSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.transferSubsystem = transferSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(intakeSubsystem, transferSubsystem, indexerSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
    transferSubsystem.intake();
    indexerSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    transferSubsystem.stop();
    indexerSubsystem.stop();
  }

}
