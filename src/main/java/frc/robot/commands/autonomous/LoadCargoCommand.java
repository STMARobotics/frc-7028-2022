package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Command that runs the intake and transfer to load cargo.
 */
public class LoadCargoCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final TransferSubsystem transferSubsystem;

  public LoadCargoCommand(
      IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.transferSubsystem = transferSubsystem;

    addRequirements(intakeSubsystem, transferSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
    transferSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    transferSubsystem.stop();
  }
  
}
