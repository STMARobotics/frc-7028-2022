package frc.robot.testCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainTestCommand extends CommandBase {

  private static final double TEST_TIME = 3;

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final Timer timer = new Timer();

  public DriveTrainTestCommand(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }
 
  @Override
  public void execute() {
    driveTrainSubsystem.arcadeDrive(.3, 0, false);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= TEST_TIME;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    timer.stop();
  }

}
