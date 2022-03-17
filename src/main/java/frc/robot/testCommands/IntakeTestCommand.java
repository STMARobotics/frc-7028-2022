package frc.robot.testCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTestCommand extends CommandBase {
  
  private static final double TEST_TIME = 3;

  private final IntakeSubsystem intakeSubsystem;
  private final Timer timer = new Timer();

  public IntakeTestCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
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
    intakeSubsystem.stop();
    timer.stop();
  }

}
