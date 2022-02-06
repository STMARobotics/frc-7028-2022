// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  private final XboxController xboxController = new XboxController(0);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();

  private final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(
    driveTrainSubsystem, () -> -xboxController.getLeftY(), xboxController::getRightX);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(xboxController, XboxController.Button.kA.value)
    .whenHeld(new RunCommand(()->shooterSubsystem.runShooter(15000), shooterSubsystem))
    .whenReleased(new RunCommand(()->shooterSubsystem.stop(), shooterSubsystem));

    new JoystickButton(xboxController, XboxController.Button.kB.value)
    .whenHeld(new RunCommand(()->shooterSubsystem.runShooter(15500), shooterSubsystem))
    .whenReleased(new RunCommand(()->shooterSubsystem.stop(), shooterSubsystem));

    new JoystickButton(xboxController, XboxController.Button.kX.value)
    .whenHeld(new RunCommand(()->shooterSubsystem.runShooter(16000), shooterSubsystem))
    .whenReleased(new RunCommand(()->shooterSubsystem.stop(), shooterSubsystem));

    new JoystickButton(xboxController, XboxController.Button.kY.value)
    .whenHeld(new RunCommand(()->shooterSubsystem.runShooter(20000), shooterSubsystem))
    .whenReleased(new RunCommand(()->shooterSubsystem.stop(), shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
