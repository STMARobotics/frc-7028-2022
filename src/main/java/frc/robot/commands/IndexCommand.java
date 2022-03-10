package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Default command for Indexer subsystem
 */
public class IndexCommand extends CommandBase {

  private final IndexerSubsystem indexerSubsystem;

  public IndexCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.intake();
  }
  
}