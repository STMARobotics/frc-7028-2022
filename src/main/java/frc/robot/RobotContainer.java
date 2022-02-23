// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.JetsonBallCommand;
import frc.robot.commands.MusicCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopDriveCommand;
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
  private final ShooterLimelightSubsystem limelightSubsystem =
      new ShooterLimelightSubsystem(LimeLightConstants.LIMELIGHT_CONFIG);

  private final IndexerCommand indexCommand = new IndexerCommand(indexerSubsystem);
  private final JetsonBallCommand jetsonBallCommand = 
      new JetsonBallCommand(driveTrainSubsystem, jetsonSubsystem, intakeSubsystem, transferSubsystem, indexerSubsystem);
  private final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(
      driveTrainSubsystem, () -> -driverController.getLeftY() / 2, () -> driverController.getRightX() / 2);
  private final ShootCommand shootCommand = 
      new ShootCommand(shooterSubsystem, limelightSubsystem, driveTrainSubsystem, indexerSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);
    SmartDashboard.putData(new MusicCommand(driveTrainSubsystem));

    // Forward ports for USB access
    PortForwarder.add(8811, "10.70.28.11", 5801); // Limelight
    PortForwarder.add(8813, "10.70.28.13", 1181); // Jetson

    configureButtonBindings();
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

    // // TODO Remove this, it's just for shop testing
    // var shooterTab  = Shuffleboard.getTab("Shooter");
    // shooterSubsystem.addDashboardWidgets(shooterTab.getLayout("shooter", BuiltInLayouts.kList));
    // var shooterSpeed = shooterTab.add("Y-Button Shoot Speed", 0).withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("Max", 21000, "Min", 0)).getEntry();
    // new JoystickButton(driverController, XboxController.Button.kY.value)
    //     .whileHeld(new JustShootCommand(shooterSubsystem, indexerSubsystem, () -> shooterSpeed.getDouble(150000)));
    // limelightSubsystem.addDashboardWidgets(shooterTab.getLayout("Limelight", BuiltInLayouts.kList));
    
    // Shooting and Limelight
    new JoystickButton(driverController, XboxController.Button.kA.value).whileHeld(shootCommand);
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .toggleWhenPressed(new StartEndCommand(limelightSubsystem::enable, limelightSubsystem::disable));
    new JoystickButton(driverController, XboxController.Button.kBack.value).toggleWhenPressed(new StartEndCommand(
            () -> limelightSubsystem.setProfile(Profile.NEAR), () -> limelightSubsystem.setProfile(Profile.FAR)));

    // Detect and Chase Cargo
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(jetsonBallCommand);

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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public Command getTeleopCommand() {
    // An ExampleCommand will run in autonomous
    return indexCommand;
  }
}
