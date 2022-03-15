// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.Collections;
import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.LoadCargoCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleopTurretCommand;
import frc.robot.commands.TrackTargetCommand;
import frc.robot.commands.UnloadCargoCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterLimelightSubsystem;
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
  private final XboxController operatorController = new XboxController(1);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final JetsonSubsystem jetsonSubsystem = new JetsonSubsystem();
  private final ShooterLimelightSubsystem limelightSubsystem = new ShooterLimelightSubsystem(
      LimeLightConstants.LIMELIGHT_CONFIG);

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(
      driveTrainSubsystem, () -> -driverController.getLeftY(), () -> driverController.getRightX());
  private final TrackTargetCommand trackTargetCommand = new TrackTargetCommand(driveTrainSubsystem::getCurrentPose);

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

    trackTargetCommand.schedule();

    // Debug code to set the robot starting position on enable
    // new Trigger(RobotState::isEnabled).whenActive(
    //     () -> driveTrainSubsystem.setCurrentPose(
    //         new Pose2d(Units.inchesToMeters(27 * 12), Units.inchesToMeters(119), Rotation2d.fromDegrees(-90))));
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

    new JoystickButton(driverController, XboxController.Button.kY.value).toggleWhenPressed(new StartEndCommand(
        () -> teleDriveCommand.setReverseMode(true), () -> teleDriveCommand.setReverseMode(false)));
    
    // Shooting and Limelight
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileHeld(new ShootCommand(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem,
            driveTrainSubsystem, trackTargetCommand::getAngleToTarget, true));

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .toggleWhenPressed(new StartEndCommand(limelightSubsystem::enable, limelightSubsystem::disable));

    new JoystickButton(driverController, XboxController.Button.kBack.value).toggleWhenPressed(new StartEndCommand(
        () -> limelightSubsystem.setProfile(Profile.NEAR), () -> limelightSubsystem.setProfile(Profile.FAR)));

    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 5000));

    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .toggleWhenPressed(new StartEndCommand(() -> turretSubsystem.positionToRobotAngle(180), turretSubsystem::stop, turretSubsystem));

    // Detect and Chase Cargo
    // new JoystickButton(driverController, XboxController.Button.kX.value)
    //    .whileHeld(new JetsonCargoCommand(driveTrainSubsystem, jetsonSubsystem, intakeSubsystem, transferSubsystem, indexerSubsystem));

    // Intake/transfer/indexer
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileHeld(new UnloadCargoCommand(intakeSubsystem, transferSubsystem, indexerSubsystem));

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileHeld(new LoadCargoCommand(intakeSubsystem, transferSubsystem));
    
    new POVButton(driverController, 0).whenPressed(intakeSubsystem::deploy);
    new POVButton(driverController, 180).whenPressed(intakeSubsystem::retract);
    new POVButton(driverController, 270).whenPressed(intakeSubsystem::toggleCompressorEnabled);
    // addShootCalibrationMode();
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    turretSubsystem.setDefaultCommand(new TeleopTurretCommand(
        turretSubsystem, driverController::getRightTriggerAxis, driverController::getLeftTriggerAxis));
    indexerSubsystem.setDefaultCommand(new IndexCommand(indexerSubsystem));
    climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem, operatorController::getLeftY, operatorController::getRightY));
  }

  /**
   * Adds "Shooter" Shuffleboard tab and Y-button binding for tuning the shooter.
   */
  private void addShootCalibrationMode() {
    var shooterTab = Shuffleboard.getTab("Shooter");
    shooterSubsystem.addDashboardWidgets(shooterTab.getLayout("shooter", BuiltInLayouts.kList));
    var shooterSpeed = shooterTab.add("Y-Button Shoot Speed", 0).withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("Max", 21000, "Min", 0)).getEntry();
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileHeld(new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> shooterSpeed.getDouble(150000)));
    limelightSubsystem.addDashboardWidgets(shooterTab.getLayout("Limelight", BuiltInLayouts.kList));
  }

  private void configureSubsystemDashboard() {
    LiveWindow.disableAllTelemetry();
    var drivetrainLayout = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(0, 0);
    driveTrainSubsystem.addDashboardWidgets(drivetrainLayout);

    var shooterLayout = Dashboard.subsystemsTab.getLayout("Shooter", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(2, 0);
    shooterSubsystem.addDashboardWidgets(shooterLayout);

    var limelightLayout = Dashboard.subsystemsTab.getLayout("Limelight", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(4, 0);
    limelightSubsystem.addDashboardWidgets(limelightLayout);

    var jetsonLayout = Dashboard.subsystemsTab.getLayout("Jetson", BuiltInLayouts.kGrid)
        .withSize(2, 2).withPosition(4, 2);
    jetsonSubsystem.addDashboardWidgets(jetsonLayout);

    var turretLayout = Dashboard.subsystemsTab.getLayout("Turret", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(6, 0);
    turretSubsystem.addDashboardWidgets(turretLayout);

    var indexerLayout = Dashboard.subsystemsTab.getLayout("Indexer", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(8, 0);
    indexerSubsystem.addDashboardWidgets(indexerLayout);
  }

  private void configureDriverDashboard() {
    var camera = new HttpCamera("Driver", "http://10.70.28.13:1182");
    if (camera != null) {
      Dashboard.driverTab.add("Driver Camera", camera).withSize(5, 3).withPosition(0, 0);
    }
    shooterSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    indexerSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    jetsonSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    trackTargetCommand.addDriverDashboardWidgets(Dashboard.driverTab);
    // Shuffleboard.selectTab(Dashboard.driverTab.getTitle());
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
        .andThen(trajectoryCommand.deadlineWith(new LoadCargoCommand(intakeSubsystem, transferSubsystem)))
        .andThen(new ShootCommand(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem,
            driveTrainSubsystem, trackTargetCommand::getAngleToTarget, 2));
  }
}
