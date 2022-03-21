package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  private final BiConsumer<RumbleType, Double> rumble;

  public LoadCargoCommand(
      IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,BooleanSupplier isIndexerFull, BiConsumer<RumbleType, Double> rumble) {
    this.intakeSubsystem = intakeSubsystem;
    this.transferSubsystem = transferSubsystem;
    this.isIndexerFull = isIndexerFull;
    this.rumble = rumble;

    addRequirements(intakeSubsystem, transferSubsystem);
  }

  @Override
  public void execute() {
    if (rumble != null) {
      if (isIndexerFull.getAsBoolean()) {
        rumble.accept(RumbleType.kLeftRumble, 1d);
      } else {
        rumble.accept(RumbleType.kLeftRumble, 0d);
      }
    }
    intakeSubsystem.intake();
    transferSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    if (rumble != null) {
      rumble.accept(RumbleType.kLeftRumble, 0d);
    }
    intakeSubsystem.stop();
    transferSubsystem.stop();
  }
  
}
