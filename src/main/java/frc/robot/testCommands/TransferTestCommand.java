package frc.robot.testCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class TransferTestCommand extends CommandBase {

  private static final double TEST_TIME = 3;

  private final TransferSubsystem transferSubsystem;
  private final Timer timer = new Timer();
  
  public TransferTestCommand(TransferSubsystem transferSubsystem) {
    this.transferSubsystem = transferSubsystem;

    addRequirements(transferSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= TEST_TIME;
  }

  @Override
  public void end(boolean interrupted) {
    transferSubsystem.stop();
    timer.stop();
  }
}
