package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TeleopDriveCommand extends CommandBase {
  
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final DoubleSupplier speedSupplier;
  private final DoubleSupplier rotationSupplier;
  
  public TeleopDriveCommand(
      DriveTrainSubsystem driveTrainSubsystem, DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.speedSupplier = speedSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void execute() {
    driveTrainSubsystem.arcadeDrive(speedSupplier.getAsDouble(), rotationSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
