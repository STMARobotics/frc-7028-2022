package frc.robot.commands;

import static frc.robot.Constants.DriverConstants.DEADBAND_HIGH;
import static frc.robot.Constants.DriverConstants.DEADBAND_LOW;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DeadbandFilter;
import frc.robot.subsystems.ClimbSubsystem;

public class TeleopClimbCommand extends CommandBase {

  private final ClimbSubsystem climbSubsystem;
  private final Supplier<Double> firstStageSupplier;
  private final DeadbandFilter deadbandFilter = new DeadbandFilter(DEADBAND_LOW, DEADBAND_HIGH);

  public TeleopClimbCommand(
      ClimbSubsystem climbSubsystem,
      Supplier<Double> firstStageSupplier) {
    this.climbSubsystem = climbSubsystem;
    this.firstStageSupplier = firstStageSupplier;
    
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    climbSubsystem.setFirstStage(deadbandFilter.calculate(firstStageSupplier.get()));
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopFirstStage();
  }
  
}
