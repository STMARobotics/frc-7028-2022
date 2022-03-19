package frc.robot.commands;

import static frc.robot.Constants.DriverConstants.DEADBAND_FILTER;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class TeleopClimbCommand extends CommandBase {

  private final ClimbSubsystem climbSubsystem;
  private final DoubleSupplier firstStageSupplier;
  private final BiConsumer<RumbleType, Double> rumble;
  private final BooleanSupplier turretIsClear;

  public TeleopClimbCommand(
      ClimbSubsystem climbSubsystem,
      DoubleSupplier firstStageSupplier,
      BiConsumer<RumbleType, Double> rumble,
      BooleanSupplier turretIsClear) {
    this.climbSubsystem = climbSubsystem;
    this.firstStageSupplier = firstStageSupplier;
    this.rumble = rumble;
    this.turretIsClear = turretIsClear;
    
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    var requestedSpeed = DEADBAND_FILTER.calculate(firstStageSupplier.getAsDouble());
    if (requestedSpeed == 0) {
      climbSubsystem.stopFirstStage();
      rumble.accept(RumbleType.kLeftRumble, 0d);
    } else {
      if (turretIsClear.getAsBoolean()) {
        // turret is clear so move (the subsystem also enforces this)
        climbSubsystem.setFirstStage(DEADBAND_FILTER.calculate(requestedSpeed));
        rumble.accept(RumbleType.kLeftRumble, 0d);
      } else {
        // turret is not clear, so do not move and rumble the controller
        climbSubsystem.stopFirstStage();
        rumble.accept(RumbleType.kLeftRumble, 1d);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopFirstStage();
    rumble.accept(RumbleType.kLeftRumble, 0d);
  }
  
}
