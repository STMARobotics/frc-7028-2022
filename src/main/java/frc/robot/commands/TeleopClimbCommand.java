package frc.robot.commands;

import static frc.robot.Constants.ClimbConstants.CLIMB_DEADBAND_FILTER;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class TeleopClimbCommand extends CommandBase {

  private final ClimbSubsystem climbSubsystem;
  private final DoubleSupplier firstStageSupplier;
  private final DoubleConsumer rumble;
  private final BooleanSupplier turretIsClear;

  public TeleopClimbCommand(
      ClimbSubsystem climbSubsystem,
      DoubleSupplier firstStageSupplier,
      DoubleConsumer rumble,
      BooleanSupplier turretIsClear) {
    this.climbSubsystem = climbSubsystem;
    this.firstStageSupplier = firstStageSupplier;
    this.rumble = rumble == null ? (r) -> {} : rumble;
    this.turretIsClear = turretIsClear;
    
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    var requestedSpeed = CLIMB_DEADBAND_FILTER.calculate(firstStageSupplier.getAsDouble());
    if (requestedSpeed == 0) {
      climbSubsystem.stopFirstStage();
      rumble.accept(0d);
    } else {
      if (turretIsClear.getAsBoolean()) {
        // turret is clear so move (the subsystem also enforces this)
        climbSubsystem.setFirstStage(CLIMB_DEADBAND_FILTER.calculate(requestedSpeed));
        rumble.accept(0d);
      } else {
        // turret is not clear, so do not move and rumble the controller
        climbSubsystem.stopFirstStage();
        rumble.accept(1d);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopFirstStage();
    rumble.accept(0d);
  }
  
}
