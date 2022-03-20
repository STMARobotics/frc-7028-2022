package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * TurnToAngleCommand
 */
public class TurnToAngleCommand extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final PIDController pidController = new PIDController(.02, 0.0, 0.0);

  public TurnToAngleCommand(double angle, DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.pidController.setTolerance(2);
    pidController.setSetpoint(angle);
    pidController.enableContinuousInput(-180, 180);

    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double rotation = -pidController.calculate(driveTrainSubsystem.getCurrentPose().getRotation().getDegrees());
    driveTrainSubsystem.arcadeDrive(0, rotation, false);
  }

  @Override
  public boolean isFinished() {
    if (pidController.atSetpoint()) {
      System.out.println("PID at setpoint: " + driveTrainSubsystem.getCurrentPose().getRotation().getDegrees());
    }

    return pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Done turning: " + driveTrainSubsystem.getCurrentPose().getRotation().getDegrees());
    driveTrainSubsystem.stop();
  }
  
}