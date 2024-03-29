// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.Constants.ClimbConstants.CLIMB_DEADBAND_FILTER;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.ClimbRaisedCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.LoadCargoTelopCommand;
import frc.robot.commands.PositionTurretCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleopClimbCommand;
import frc.robot.commands.TeleopTurretCommand;
import frc.robot.commands.TrackTargetCommand;
import frc.robot.commands.UnloadCargoTeleopCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final XboxController operatorController = new XboxController(2);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(turretSubsystem::isClearOfClimb);
  private final ShooterLimelightSubsystem limelightSubsystem = new ShooterLimelightSubsystem(
      LimeLightConstants.LIMELIGHT_CONFIG);

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(
      driveTrainSubsystem, () -> -driverController.getLeftY(), () -> driverController.getRightX());

  private final LoadCargoTelopCommand loadCargo = new LoadCargoTelopCommand(intakeSubsystem, transferSubsystem);
  private final UnloadCargoTeleopCommand unloadCargo = new UnloadCargoTeleopCommand(intakeSubsystem, transferSubsystem, indexerSubsystem);

  private final TeleopClimbCommand teleopClimbCommand = new TeleopClimbCommand(
      climbSubsystem, () -> -operatorController.getLeftY(),
      (r) -> operatorController.setRumble(RumbleType.kRightRumble, r),
      turretSubsystem::isClearOfClimb);

  private final TrackTargetCommand trackTargetCommand = new TrackTargetCommand(driveTrainSubsystem::getCurrentPose);

  private final AutonomousBuilder autoBuilder = new AutonomousBuilder(
    driveTrainSubsystem, indexerSubsystem, intakeSubsystem, limelightSubsystem, shooterSubsystem, transferSubsystem, 
    turretSubsystem, trackTargetCommand);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Forward ports for USB access
    PortForwarder.add(8811, "10.70.28.11", 5801); // Limelight
    PortForwarder.add(8813, "10.70.28.13", 1182); // Jetson

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

    var toggleReverse = new StartEndCommand(
        () -> teleDriveCommand.setReverseMode(true), () -> teleDriveCommand.setReverseMode(false));
    new JoystickButton(driverController, XboxController.Button.kX.value).toggleWhenPressed(toggleReverse);
    new JoystickButton(driverController, XboxController.Button.kA.value).toggleWhenPressed(toggleReverse);
  
    // Shooting
    var shootCommand = new ShootCommand(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem,
        driveTrainSubsystem, trackTargetCommand::getAngleToTarget, true,
        (r) -> driverController.setRumble(RumbleType.kRightRumble, r));
    new Trigger(() -> driverController.getRightTriggerAxis() > .5)
        .whileActiveContinuous(shootCommand.alongWith(new RumbleCommand(operatorController, RumbleType.kLeftRumble)));

    // Limelight
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .toggleWhenPressed(new StartEndCommand(limelightSubsystem::enable, limelightSubsystem::disable));
    
    // Bloop
    new Trigger(() -> driverController.getLeftTriggerAxis() > .5)
        .whileActiveContinuous(new RumbleCommand(operatorController, RumbleType.kLeftRumble)
            .alongWith(new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> 11000))
            .alongWith(new PositionTurretCommand(turretSubsystem, 180)));

    // Operator
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
        .whenPressed(loadCargo);

    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        .whenPressed(() -> CommandScheduler.getInstance().cancel(loadCargo))
        .whenPressed(() -> CommandScheduler.getInstance().cancel(unloadCargo));

    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .whenPressed(unloadCargo);

    // Position the turret to 180 degrees when the climb is up or the operator is trying to move the climb
    new Trigger(
        () -> climbSubsystem.isFirstStageRaised() || CLIMB_DEADBAND_FILTER.calculate(operatorController.getLeftY()) != 0)
      .whileActiveContinuous(new ClimbRaisedCommand(turretSubsystem, limelightSubsystem, shooterSubsystem));

    new JoystickButton(operatorController, XboxController.Button.kBack.value)
        .whileHeld(new StartEndCommand(climbSubsystem::disableLimits, climbSubsystem::resetAndEnableLimits));
    // addShootCalibrationMode();

    new JoystickButton(operatorController, XboxController.Button.kB.value)
        .whenPressed(limelightSubsystem::takeSnapshot);
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    turretSubsystem.setDefaultCommand(new TeleopTurretCommand(
        turretSubsystem, operatorController::getRightTriggerAxis, operatorController::getLeftTriggerAxis));
    indexerSubsystem.setDefaultCommand(new IndexCommand(indexerSubsystem));
    climbSubsystem.setDefaultCommand(teleopClimbCommand);
    // shooterSubsystem.setDefaultCommand(
    //     new RunCommand(() -> shooterSubsystem.prepareToShoot(MIN_DISTANCE), shooterSubsystem));
  }

  public void teleopInit() {
    intakeSubsystem.retract();
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

    var turretLayout = Dashboard.subsystemsTab.getLayout("Turret", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(6, 0);
    turretSubsystem.addDashboardWidgets(turretLayout);

    var indexerLayout = Dashboard.subsystemsTab.getLayout("Indexer", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(8, 0);
    indexerSubsystem.addDashboardWidgets(indexerLayout);

    var climbLayout = Dashboard.subsystemsTab.getLayout("Climb", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 1, "Number of rows", 2))
      .withSize(2, 2).withPosition(10, 0);
    climbSubsystem.addDashboardWidgets(climbLayout);
  }

  private void configureDriverDashboard() {
    shooterSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    indexerSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    trackTargetCommand.addDriverDashboardWidgets(Dashboard.driverTab);
    climbSubsystem.addDriverDashboardWidgets(Dashboard.driverTab);
    autoBuilder.addDashboardWidgets(Dashboard.driverTab);

    Dashboard.driverTab.add(new HttpCamera("Driver", "http://10.70.28.13:1182"))
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("showCrosshair", true, "showControls", false))
        .withSize(8, 5).withPosition(0, 0);
    
    Dashboard.driverTab.add(new HttpCamera("limelight", "http://10.70.28.11:5800"))
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("showCrosshair", false, "showControls", false))
        .withSize(4, 3).withPosition(9, 2);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoBuilder.getAutonomousCommand();
  }
}
