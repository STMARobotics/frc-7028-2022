package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Command to load cargo in autonomous that also runs the indexer.
 * This is needed because a command group inherits all of its requirements.
 * That means an autonomous routine that uses the indexer for shooting will
 * not run the indexer's default command.
 */
public class LoadCargoAutoCommand extends LoadCargoCommand {
  
  private final IndexerSubsystem indexerSubsystem;

  public LoadCargoAutoCommand(
        IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IndexerSubsystem indexerSubsystem) {

    super(intakeSubsystem, transferSubsystem);
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    super.execute();
    indexerSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    indexerSubsystem.stop();
  }

}
