package frc.robot.testCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerTestCommand extends CommandBase {
  
  private static final double TEST_TIME = 3;

  private final IndexerSubsystem indexerSubsystem;
  private final Timer timer = new Timer();

  public IndexerTestCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
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
    indexerSubsystem.stop();
    timer.stop();
  }
}
