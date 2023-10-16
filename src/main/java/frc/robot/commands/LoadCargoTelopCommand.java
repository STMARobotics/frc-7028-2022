package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * Deploys and runs the intake and transfer wheels
 */
public class LoadCargoTelopCommand extends CommandBase {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TransferSubsystem transferSubsystem;
  
  public LoadCargoTelopCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.transferSubsystem = transferSubsystem;

    addRequirements(intakeSubsystem, transferSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.deploy();
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
    transferSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.retract();
    intakeSubsystem.stop();
    transferSubsystem.stop();
  }

}
