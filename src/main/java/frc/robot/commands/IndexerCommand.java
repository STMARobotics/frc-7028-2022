package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Default command for Indexer subsystem
 */
public class IndexerCommand extends CommandBase {

  private final IndexerSubsystem indexerSubsystem;

  public IndexerCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.load();
  }
  
}