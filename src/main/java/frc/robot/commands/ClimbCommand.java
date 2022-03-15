package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {

  private final ClimbSubsystem climbSubsystem;
  private final Supplier<Double> firstStageSupplier;
  private final Supplier<Double> secondStageSupplier;

  public ClimbCommand(ClimbSubsystem climbSubsystem, Supplier<Double> firstStageSupplier, Supplier<Double> secondStageSupplier) {
    this.climbSubsystem = climbSubsystem;
    this.firstStageSupplier = firstStageSupplier;
    this.secondStageSupplier = secondStageSupplier;
    
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    climbSubsystem.setFirstStage(firstStageSupplier.get());
    climbSubsystem.setSecondStage(secondStageSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopFirstStage();
    climbSubsystem.stopSecondStage();
  }
  
}
