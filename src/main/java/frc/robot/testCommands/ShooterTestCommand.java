package frc.robot.testCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTestCommand extends CommandBase {
  
  private static final double TEST_TIME = 3;

  private final ShooterSubsystem shooterSubsystem;
  private final Timer timer = new Timer();

  public ShooterTestCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
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
    shooterSubsystem.stop();
    timer.stop();
  }
}
