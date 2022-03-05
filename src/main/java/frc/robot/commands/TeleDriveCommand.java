package frc.robot.commands;

import static frc.robot.Constants.DriverConstants.DEADBAND_HIGH;
import static frc.robot.Constants.DriverConstants.DEADBAND_LOW;
import static frc.robot.Constants.DriverConstants.ROTATE_RATE_LIMIT_ARCADE;
import static frc.robot.Constants.DriverConstants.SLOW_MODE_ROTATION_MULTIPLIER;
import static frc.robot.Constants.DriverConstants.SLOW_MODE_SPEED_MULTIPLIER;
import static frc.robot.Constants.DriverConstants.SPEED_RATE_LIMIT_ARCADE;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DeadbandFilter;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * TeleDriveCommand
 */
public class TeleDriveCommand extends CommandBase {

  private final DeadbandFilter deadbandFilter = new DeadbandFilter(DEADBAND_LOW, DEADBAND_HIGH);

  private final DoubleSupplier speedSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DriveTrainSubsystem driveTrainSubsystem;
  
  private final SlewRateLimiter speedRateLimiter = new SlewRateLimiter(SPEED_RATE_LIMIT_ARCADE);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATE_RATE_LIMIT_ARCADE);

  private NetworkTableEntry cameraFrontEntry = 
      NetworkTableInstance.getDefault().getTable("DriverCam").getEntry("Front");

  private boolean slowMode = false;
  private boolean reverseMode = false;

  public TeleDriveCommand(
      DriveTrainSubsystem driveTrainSubsystem, DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.speedSupplier = speedSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    // Set the rate limiters to the current drivetrain speeds. The robot may not be standing still and we want changes
    // to be limited from the current speeds, not from zero.
    speedRateLimiter.reset(driveTrainSubsystem.getVelocityPercent());
    rotationRateLimiter.reset(driveTrainSubsystem.getAngularVelocityPercent());
  }

  @Override
  public void execute() {
    driveTrainSubsystem.arcadeDrive(getSpeed(), getRotation(), true);
  }

  private double getSpeed() {
    double speed = deadbandFilter.calculate(speedSupplier.getAsDouble());
    if (isSlowMode()) {
      speed *= SLOW_MODE_SPEED_MULTIPLIER;
    }
    if (isReverseMode()) {
      speed = -speed;
    }
    return speedRateLimiter.calculate(speed);
  }

  private double getRotation() {
    double rotation = deadbandFilter.calculate(rotationSupplier.getAsDouble());
    if (isSlowMode()) {
      rotation *= SLOW_MODE_ROTATION_MULTIPLIER;
    }
    return rotationRateLimiter.calculate(rotation);
  }

  public void toggleSlowMode() {
    slowMode = !slowMode;
  }

  public boolean isSlowMode() {
    return slowMode;
  }

  public void toggleReverseMode() {
    setReverseMode(!isReverseMode());
  }

  public void setReverseMode(boolean reverse) {
    reverseMode = reverse;
    cameraFrontEntry.setBoolean(!reverseMode);
  }

  public boolean isReverseMode() {
    return reverseMode;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    speedRateLimiter.reset(0);
    rotationRateLimiter.reset(0);
  }

}