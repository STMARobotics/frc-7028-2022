package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Command that runs the intake and transfer to load cargo.
 */
public class LoadCargoCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final TransferSubsystem transferSubsystem;
  private final BooleanSupplier isIndexerFull;
  private final DoubleConsumer rumble;

  public LoadCargoCommand(
      IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,
      BooleanSupplier isIndexerFull, DoubleConsumer rumble) {
    this.intakeSubsystem = intakeSubsystem;
    this.transferSubsystem = transferSubsystem;
    this.isIndexerFull = isIndexerFull;
    this.rumble = rumble == null ? (r) -> {} : rumble;

    addRequirements(intakeSubsystem, transferSubsystem);
  }

  @Override
  public void execute() {
    rumble.accept(isIndexerFull.getAsBoolean() ? 1d : 0d);
    intakeSubsystem.intake();
    transferSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    rumble.accept(0d);
    intakeSubsystem.stop();
    transferSubsystem.stop();
  }
  
}
