// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.Collections;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.JetsonCargoCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

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
  private final JetsonSubsystem jetsonSubsystem = new JetsonSubsystem();
  private final ShooterLimelightSubsystem limelightSubsystem = new ShooterLimelightSubsystem(
      LimeLightConstants.LIMELIGHT_CONFIG);

  private final JetsonCargoCommand jetsonCargoCommand = new JetsonCargoCommand(driveTrainSubsystem, jetsonSubsystem,
      intakeSubsystem, transferSubsystem, indexerSubsystem);
  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(
      driveTrainSubsystem, () -> -driverController.getLeftY(), () -> driverController.getRightX());
  private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem, limelightSubsystem, driveTrainSubsystem,
      indexerSubsystem, transferSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Forward ports for USB access
    PortForwarder.add(8811, "10.70.28.11", 5801); // Limelight
    PortForwarder.add(8813, "10.70.28.13", 1181); // Jetson

    configureButtonBindings();
    configureSubsystemCommands();
    configureSubsystemDashboard();
    configureDriverDashboard();
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
    // Drivetrain
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whenPressed(teleDriveCommand::toggleSlowMode);

    // Shooting and Limelight
    new JoystickButton(driverController, XboxController.Button.kA.value).whileHeld(shootCommand);

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .toggleWhenPressed(new StartEndCommand(limelightSubsystem::enable, limelightSubsystem::disable));

    new JoystickButton(driverController, XboxController.Button.kBack.value).toggleWhenPressed(new StartEndCommand(
        () -> limelightSubsystem.setProfile(Profile.NEAR), () -> limelightSubsystem.setProfile(Profile.FAR)));

    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileHeld(new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 5000));

    // Detect and Chase Cargo
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(jetsonCargoCommand);

    // Intake/transfer/indexer
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileHeld(intakeSubsystem::reverse, intakeSubsystem)
        .whileHeld(indexerSubsystem::unload, indexerSubsystem)
        .whileHeld(transferSubsystem::output, transferSubsystem)
        .whenReleased(intakeSubsystem::stop, intakeSubsystem)
        .whenReleased(indexerSubsystem::stop, indexerSubsystem)
        .whenReleased(transferSubsystem::stop, transferSubsystem);

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileHeld(intakeSubsystem::intake, intakeSubsystem)
        .whileHeld(indexerSubsystem::load, indexerSubsystem)
        .whileHeld(transferSubsystem::intake, transferSubsystem)
        .whenReleased(intakeSubsystem::stop, intakeSubsystem)
        .whenReleased(indexerSubsystem::stop, indexerSubsystem)
        .whenReleased(transferSubsystem::stop, transferSubsystem);
  }

  /**
   * Adds "Shooter" Shuffleboard tab and Y-button binding for tuning the shooter.
   */
  private void addShootCalibrationMode() {
    var shooterTab = Shuffleboard.getTab("Shooter");
    shooterSubsystem.addDashboardWidgets(shooterTab.getLayout("shooter", BuiltInLayouts.kList));
    var shooterSpeed = shooterTab.add("Y-Button Shoot Speed", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Max", 21000, "Min", 0)).getEntry();
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileHeld(new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> shooterSpeed.getDouble(150000)));
    limelightSubsystem.addDashboardWidgets(shooterTab.getLayout("Limelight", BuiltInLayouts.kList));
  }

  private void configureSubsystemDashboard() {
    var drivetrainLayout = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(0, 0);
    driveTrainSubsystem.addDashboardWidgets(drivetrainLayout);
    drivetrainLayout.add(driveTrainSubsystem);

    var shooterLayout = Dashboard.subsystemsTab.getLayout("Shooter", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(2, 0);
    shooterSubsystem.addDashboardWidgets(shooterLayout);
    shooterLayout.add(shooterSubsystem);

    var limelightLayout = Dashboard.subsystemsTab.getLayout("Limelight", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(4, 0);
    limelightSubsystem.addDashboardWidgets(limelightLayout);
    limelightLayout.add(limelightSubsystem);

    var jetsonLayout = Dashboard.subsystemsTab.getLayout("Jetson", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(6, 0);
    jetsonSubsystem.addDashboardWidgets(jetsonLayout);
    jetsonLayout.add(jetsonSubsystem);
  }

  private void configureDriverDashboard() {
    jetsonSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    Shuffleboard.selectTab(Dashboard.driverTab.getTitle());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var endPose = new Pose2d(inchesToMeters(72), 0, new Rotation2d(0));
    var trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(),
        Collections.emptyList(),
        endPose,
        new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO * .5)
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT));
    var trajectoryCommand = driveTrainSubsystem.createCommandForTrajectory(trajectory);

    return new InstantCommand(() -> driveTrainSubsystem.setCurrentPose(new Pose2d()), driveTrainSubsystem)
        .andThen(trajectoryCommand
            .raceWith(new StartEndCommand(intakeSubsystem::intake, intakeSubsystem::stop, intakeSubsystem))
            .raceWith(new StartEndCommand(transferSubsystem::intake, transferSubsystem::stop, transferSubsystem))
            .raceWith(new StartEndCommand(indexerSubsystem::load, indexerSubsystem::stop, indexerSubsystem)))
        .andThen(
            new StartEndCommand(indexerSubsystem::unload, indexerSubsystem::stop, indexerSubsystem).withTimeout(.1))
        .andThen(new ShootCommand(shooterSubsystem, limelightSubsystem, driveTrainSubsystem, indexerSubsystem,
            transferSubsystem));
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
  }
}
