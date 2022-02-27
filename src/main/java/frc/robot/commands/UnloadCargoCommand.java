package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Command that runs the intake, transfer, and indexer to unload/reverse cargo back out onto the field.
 */
public class UnloadCargoCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final TransferSubsystem transferSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public UnloadCargoCommand(
      IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IndexerSubsystem indexerSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.transferSubsystem = transferSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(intakeSubsystem, transferSubsystem, indexerSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.reverse();
    transferSubsystem.output();
    indexerSubsystem.unload();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    transferSubsystem.stop();
    indexerSubsystem.stop();
  }
  
}
