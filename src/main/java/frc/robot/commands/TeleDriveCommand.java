package frc.robot.commands;

import static frc.robot.Constants.DriverConstants.DEADBAND_HIGH;
import static frc.robot.Constants.DriverConstants.DEADBAND_LOW;
import static frc.robot.Constants.DriverConstants.ROTATION_MULTIPLIER;
import static frc.robot.Constants.DriverConstants.SLOW_MODE_ROTATION_MULTIPLIER;
import static frc.robot.Constants.DriverConstants.SLOW_MODE_SPEED_MULTIPLIER;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DeadbandFilter;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * TeleDriveCommand
 */
public class TeleDriveCommand extends CommandBase {

    private final DeadbandFilter deadbandFilter = new DeadbandFilter(DEADBAND_LOW, DEADBAND_HIGH);

    private final XboxController driverController;
    private final DriveTrainSubsystem driveTrainSubsystem;
    private boolean slowMode = false;
    private boolean reverseMode = false;

    public TeleDriveCommand(XboxController driverController, DriveTrainSubsystem driveTrainSubsystem) {
        this.driverController = driverController;
        this.driveTrainSubsystem = driveTrainSubsystem;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void execute() {
        driveTrainSubsystem.arcadeDrive(getSpeed(), getRotation(), true);
    }

    private double getSpeed() {
        double speed = deadbandFilter.calculate(-driverController.getLeftY());
        if (isSlowMode()) {
            speed *= SLOW_MODE_SPEED_MULTIPLIER;
        }
        if (isReverseMode()) {
            speed = -speed;
        }
        return speed;
    }

    private double getRotation() {
        double rotation = deadbandFilter.calculate(driverController.getRightX()) * ROTATION_MULTIPLIER;
        if (isSlowMode()) {
            rotation *= SLOW_MODE_ROTATION_MULTIPLIER;
        }
        return rotation;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void toggleReverseMode() {
        reverseMode = !reverseMode;
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
    }

}