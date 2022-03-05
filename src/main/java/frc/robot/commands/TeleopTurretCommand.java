package frc.robot.commands;

import static frc.robot.Constants.DriverConstants.DEADBAND_HIGH;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.DeadbandFilter;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to manually run the turret.
 */
public class TeleopTurretCommand extends CommandBase {
  private final DeadbandFilter deadbandFilter = new DeadbandFilter(DriverConstants.DEADBAND_LOW, DEADBAND_HIGH);
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
    speed = deadbandFilter.calculate(speed);
    speed *= Math.abs(speed);
    turretSubsystem.run(speed / 5);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

}
