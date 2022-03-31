// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.LoadCargoCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleopTurretCommand;
import frc.robot.commands.TrackTargetCommand;
import frc.robot.commands.UnloadCargoCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController driverController = new XboxController(0);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(
      driveTrainSubsystem, () -> driverController.getLeftY(), () -> driverController.getRightX());

  private final TrackTargetCommand trackTargetCommand = new TrackTargetCommand(driveTrainSubsystem::getCurrentPose);

  private final Boolean indoorMode = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Forward ports for USB access
    PortForwarder.add(8811, "10.70.28.11", 5801); // Limelight
    PortForwarder.add(8813, "10.70.28.13", 1182); // Jetson

    teleDriveCommand.toggleSlowMode();

    configureButtonBindings();
    configureSubsystemCommands();
    configureSubsystemDashboard();
    configureDriverDashboard();

    trackTargetCommand.schedule();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver
    // Drivetrain
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whenPressed(teleDriveCommand::toggleSlowMode);

    new JoystickButton(driverController, XboxController.Button.kA.value).whileActiveContinuous(
        new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 11000));

    if (!indoorMode) {
      new JoystickButton(driverController, XboxController.Button.kB.value).whileActiveContinuous(
          new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 13000));

      new JoystickButton(driverController, XboxController.Button.kY.value).whileActiveContinuous(
          new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 16000));

      new JoystickButton(driverController, XboxController.Button.kX.value).whileActiveContinuous(
          new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 19000));
    }

    // Intake/transfer/indexer
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileHeld(new UnloadCargoCommand(intakeSubsystem, transferSubsystem, indexerSubsystem));

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).whileHeld(new LoadCargoCommand(
        intakeSubsystem, transferSubsystem, indexerSubsystem::isFullSensorTripped,
        (r) -> driverController.setRumble(RumbleType.kRightRumble, r)));

    new POVButton(driverController, 0).whenPressed(intakeSubsystem::deploy);
    new POVButton(driverController, 180).whenPressed(intakeSubsystem::retract);
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    turretSubsystem.setDefaultCommand(new TeleopTurretCommand(
        turretSubsystem, driverController::getRightTriggerAxis, driverController::getLeftTriggerAxis));
    indexerSubsystem.setDefaultCommand(new IndexCommand(indexerSubsystem));
  }

  /**
   * Adds "Shooter" Shuffleboard tab and Y-button binding for tuning the shooter.
   */

  private void configureSubsystemDashboard() {
    LiveWindow.disableAllTelemetry();
    var drivetrainLayout = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(0, 0);
    driveTrainSubsystem.addDashboardWidgets(drivetrainLayout);

    var shooterLayout = Dashboard.subsystemsTab.getLayout("Shooter", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(2, 0);
    shooterSubsystem.addDashboardWidgets(shooterLayout);

    var turretLayout = Dashboard.subsystemsTab.getLayout("Turret", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(6, 0);
    turretSubsystem.addDashboardWidgets(turretLayout);

    var indexerLayout = Dashboard.subsystemsTab.getLayout("Indexer", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(8, 0);
    indexerSubsystem.addDashboardWidgets(indexerLayout);
  }

  private void configureDriverDashboard() {
    shooterSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    driveTrainSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    indexerSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    trackTargetCommand.addDriverDashboardWidgets(Dashboard.driverTab);
    var camera = new HttpCamera("Driver", "http://10.70.28.13:1182");
    if (camera != null) {
      Dashboard.driverTab.add("Driver Camera", camera).withSize(8, 5).withPosition(0, 0);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return teleDriveCommand;
  }
}
