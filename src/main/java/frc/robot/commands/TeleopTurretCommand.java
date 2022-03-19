package frc.robot.commands;

import static frc.robot.Constants.DriverConstants.DEADBAND_FILTER;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to manually run the turret.
 */
public class TeleopTurretCommand extends CommandBase {
  private final DoubleSupplier rightSupplier;
  private final DoubleSupplier leftSupplier;
  private final TurretSubsystem turretSubsystem;

  public TeleopTurretCommand(TurretSubsystem turretSubsystem, DoubleSupplier rightSupplier, DoubleSupplier leftSupplier) {
    this.turretSubsystem = turretSubsystem;
    this.rightSupplier = rightSupplier;
    this.leftSupplier = leftSupplier;

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    double speed;
    if (rightSupplier.getAsDouble() > leftSupplier.getAsDouble()) {
      speed = rightSupplier.getAsDouble();
    } else {
      speed = -leftSupplier.getAsDouble();
    }
    speed = DEADBAND_FILTER.calculate(speed);
    speed *= Math.abs(speed);
    turretSubsystem.run(speed);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

}
