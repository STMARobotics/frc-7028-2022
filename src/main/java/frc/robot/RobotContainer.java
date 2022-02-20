// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.JetsonBallCommand;
import frc.robot.commands.MusicCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController driverController = new XboxController(0);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final JetsonSubsystem jetsonSubsystem = new JetsonSubsystem();
  
  private final IndexerCommand indexCommand = new IndexerCommand(indexerSubsystem);
  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(
      driveTrainSubsystem, () -> -driverController.getLeftY(), driverController::getRightX);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData(new MusicCommand(driveTrainSubsystem));

    configureButtonBindings();
    configureSubsystemCommands();
    configureSubsystemDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drivetrain
    // Detect and Chase Cargo
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(new JetsonBallCommand(driveTrainSubsystem, jetsonSubsystem));
    
    // Shooter
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileHeld(() -> shooterSubsystem.runShooter(15000), shooterSubsystem)
        .whenReleased(() -> shooterSubsystem.stop(), shooterSubsystem);

    // Intake/transfer/indexer
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileHeld(intakeSubsystem::reverse, intakeSubsystem)
        .whileHeld(indexerSubsystem::output, indexerSubsystem)
        .whileHeld(transferSubsystem::output, transferSubsystem)
        .whenReleased(intakeSubsystem::stop, intakeSubsystem)
        .whenReleased(indexerSubsystem::stop, indexerSubsystem)
        .whenReleased(transferSubsystem::stop, transferSubsystem);
        
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileHeld(intakeSubsystem::forward, intakeSubsystem)
        .whileHeld(indexerSubsystem::intake, indexerSubsystem)
        .whileHeld(transferSubsystem::intake, transferSubsystem)
        .whenReleased(intakeSubsystem::stop, intakeSubsystem)
        .whenReleased(indexerSubsystem::stop, indexerSubsystem)
        .whenReleased(transferSubsystem::stop, transferSubsystem);
  }

  private void configureSubsystemDashboard() {
    var drivetrainLayout = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kList)
        .withSize(2, 5).withPosition(0, 0);
    driveTrainSubsystem.addDashboardWidgets(drivetrainLayout);
    drivetrainLayout.add(driveTrainSubsystem);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
  }
}
