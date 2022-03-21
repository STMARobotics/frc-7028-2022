package frc.robot;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.TrajectoryConstants.MAX_ACCELERATION_AUTO;
import static frc.robot.Constants.TrajectoryConstants.MAX_SPEED_AUTO;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.LoadCargoAutoCommand;
import frc.robot.commands.LoadCargoCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinupShooterCommand;
import frc.robot.commands.TrackTargetCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Builds all of the auto routines for the driver to choose.
 */
public class AutonomousBuilder {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TransferSubsystem transferSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final TrackTargetCommand trackTargetCommand;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public AutonomousBuilder(
      DriveTrainSubsystem driveTrainSubsystem,
      IndexerSubsystem indexerSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterLimelightSubsystem limelightSubsystem,
      ShooterSubsystem shooterSubsystem,
      TransferSubsystem transferSubsystem,
      TurretSubsystem turretSubsystem,
      TrackTargetCommand trackTargetCommand) {

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.transferSubsystem = transferSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.trackTargetCommand = trackTargetCommand;

    buildTarmacOne();
    buildTarmacTwo();
    buildTarmacThree();
    buildFourCargo();
    buildFiveCargo();
  }

  /**
   * Adds the auto chooser
   * @param dashboard dashboard
   */
  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Autonomous", autoChooser).withSize(2, 1).withPosition(6, 2);
  }

  /**
   * Gets the currently selected auto command
   * @return auto command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void buildTarmacOne() {
    var command = drivePickupShootTwo(
      new Pose2d(inchesToMeters(72), inchesToMeters(15), Rotation2d.fromDegrees(0)),
      new Pose2d(inchesToMeters(148), inchesToMeters(15), Rotation2d.fromDegrees(0)));
    autoChooser.setDefaultOption("Tarmac 1", command);
  }

  private void buildTarmacTwo() {
    var command = drivePickupShootTwo(
      new Pose2d(inchesToMeters(242), inchesToMeters(96), Rotation2d.fromDegrees(-153)),
      new Pose2d(inchesToMeters(193.83), inchesToMeters(73.95), Rotation2d.fromDegrees(-153)));
    autoChooser.addOption("Tarmac 2", command);
  }

  private void buildTarmacThree() {
    var command = drivePickupShootTwo(
      new Pose2d(7.586, inchesToMeters(71.75), Rotation2d.fromDegrees(-90)),
      new Pose2d(7.586, inchesToMeters(31.5), Rotation2d.fromDegrees(-90)));
    autoChooser.addOption("Tarmac 3", command);
  }

  /**
   * Builds a 4-cargo auto routine.
   * - Starts with one cargo at startPose
   * - Drives from startPose to cargoOnePose while running the intake
   * - Waits up to two seconds for two cargo to load while running the intake
   * - Shoots two cargo
   * - Drives from cargoOnePose to cargoTwoPose with intake running
   * - Waits up to two seconds for two cargo to load while running intake
   * - Drives from cargoTwoPose to shootPose
   * - Shoots two
   */
  private void buildFourCargo() {
    var startPose = new Pose2d(inchesToMeters(242), inchesToMeters(96), Rotation2d.fromDegrees(-153));
    var cargoOnePose = new Pose2d(inchesToMeters(193.83), inchesToMeters(73.95), Rotation2d.fromDegrees(-153));
    var angleAfterCargoOne = 180d;
    var cargoTwoPose = new Pose2d(inchesToMeters(48.36), inchesToMeters(49), Rotation2d.fromDegrees(-145));
    var shootPose = new Pose2d(inchesToMeters(193.83), inchesToMeters(73.95), Rotation2d.fromDegrees(170));

    var command = drivePickupShootTwo(startPose, cargoOnePose)
        .andThen(turnToAngle(angleAfterCargoOne))
        .andThen(drive(withSpeedAndAcceleration(1, .75), new Pose2d(cargoOnePose.getX(), cargoOnePose.getY(), Rotation2d.fromDegrees(angleAfterCargoOne)), cargoTwoPose)
            .deadlineWith(loadCargoWithIndexer())
        .andThen(new PrintCommand("Done driving to human player station"))
        .andThen(waitForCargoCount(2).withTimeout(2)))
        .andThen(new PrintCommand("Done waiting for two cargo at human player station"))
        .andThen(drive(withSpeedAndAcceleration(1, 1).setReversed(true), cargoTwoPose, shootPose)
            .deadlineWith(spinUpShooter(inchesToMeters(92)), prepareIndexerToShoot(), loadCargoWithoutIndexer()))
        .andThen(new PrintCommand("Done driving to shoot"))
        .andThen(shoot(Integer.MAX_VALUE));
    
    autoChooser.addOption("4-cargo", command);
  }

 /**
   * Builds a 5-cargo auto routine.
   * - Starts with one cargo at startPose
   * - Drives from startPose to cargoOnePose while running the intake
   * - Waits up to two seconds for two cargo to load while running the intake
   * - Shoots two cargo
   * - Drives from cargoOnePose to cargoTwoPose with intake running
   * - Waits up to two seconds for one cargo to load while running intake
   * - Shoots one cargo
   * - Drives from cargoTwoPose to cargoThreePose
   * - Waits up to two seconds for two cargo to load while running intake
   * - Drives from cargoThreePose to shootPose
   * - Shoots two
   */
  private void buildFiveCargo() {
    var startPose = new Pose2d(inchesToMeters(310), inchesToMeters(71.75), Rotation2d.fromDegrees(-90)); 
    var cargoOnePose = new Pose2d(inchesToMeters(310), inchesToMeters(31.5), Rotation2d.fromDegrees(-90));
    var angleAfterCargoOne = 125d;
    var cargoTwoPose = new Pose2d(inchesToMeters(209), inchesToMeters(86), Rotation2d.fromDegrees(-175));
    var cargoThreePose = new Pose2d(inchesToMeters(60), inchesToMeters(50), Rotation2d.fromDegrees(-151));
    var shootPose = new Pose2d(inchesToMeters(190), inchesToMeters(86), Rotation2d.fromDegrees(-175));

    var command = drivePickupShootTwo(startPose, cargoOnePose)
        .andThen(turnToAngle(angleAfterCargoOne))
        .andThen(drive(withSpeedAndAcceleration(.7, .7), new Pose2d(cargoOnePose.getX(), cargoOnePose.getY(), Rotation2d.fromDegrees(angleAfterCargoOne)), cargoTwoPose)
            .deadlineWith(loadCargoWithIndexer()))
        .andThen(new PrintCommand("Done driving to ball two"))
        .andThen(shoot(1).withTimeout(4))
        .andThen(drive(withSpeedAndAcceleration(1, .75), cargoTwoPose, cargoThreePose).deadlineWith(loadCargoWithIndexer())
        .andThen(new PrintCommand("Done driving to human player"))
        .andThen(waitForCargoCount(2).withTimeout(2)))
        .andThen(drive(withSpeedAndAcceleration(1, 1).setReversed(true), cargoThreePose, shootPose))
        .andThen(shoot(Integer.MAX_VALUE));
      
    autoChooser.addOption("5-cargo", command);
  }

  /**
   * Builds the basic routine
   * - start with one cargo
   * - drive forward while running the intake
   * - pick up a cargo
   * - shoots two
   * @param startPose starting pose
   * @param endPose ending pose
   * @return command for the routine
   */
  private Command drivePickupShootTwo(Pose2d startPose, Pose2d endPose) {
    return new PrintCommand("Starting auto")
        .alongWith(setCargoCount(1)).alongWith(deployIntake())
        .andThen(drive(withSpeedAndAcceleration(1, 1), startPose, endPose)
            .deadlineWith(loadCargoWithIndexer(), spinUpShooter(inchesToMeters(92))))
        .andThen(new PrintCommand("Done driving"))
        .andThen(shoot(2).withTimeout(3))
        .andThen(new PrintCommand("Done shooting 2"));
  }

  private Command deployIntake() {
    return new InstantCommand(intakeSubsystem::deploy);
  }

  private Command drive(TrajectoryConfig config, Pose2d... waypoints) {
    return new InstantCommand(() -> driveTrainSubsystem.setCurrentPose(waypoints[0]), driveTrainSubsystem)
        .andThen(driveTrainSubsystem.createCommandForTrajectory(generateTrajectory(Arrays.asList(waypoints), config)));
  }

  private Command spinUpShooter(double distance) {
    return new SpinupShooterCommand(shooterSubsystem, distance);
  }

  private Command prepareIndexerToShoot() {
    return new RunCommand(indexerSubsystem::prepareToShoot, indexerSubsystem);
  }

  /**
   * Makes a trajectory config for driving our robot at the given max speed and acceleration.
   * @param maxSpeedPercent max speed as a percentage of the max constant
   * @param maxAccelerationPercent max acceleration as a percentage of the max constant
   * @return trajectory config
   */
  private TrajectoryConfig withSpeedAndAcceleration(double maxSpeedPercent, double maxAccelerationPercent) {
    return new TrajectoryConfig(
        MAX_SPEED_AUTO * clamp(maxSpeedPercent, 0, 1), MAX_ACCELERATION_AUTO * clamp(maxAccelerationPercent, 0, 1))
      .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
      .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT);
  }

  private Command setCargoCount(int cargoCount) {
    return new InstantCommand(() -> indexerSubsystem.resetCargoCount(cargoCount));
  }

  private Command shoot(int cargoCount) {
    return new ShootCommand(shooterSubsystem, limelightSubsystem, turretSubsystem, indexerSubsystem,
        driveTrainSubsystem, trackTargetCommand::getAngleToTarget, cargoCount).deadlineWith(loadCargoWithoutIndexer());
  }

  private Command loadCargoWithIndexer() {
    return new LoadCargoAutoCommand(intakeSubsystem, transferSubsystem, indexerSubsystem);
  }

  private Command loadCargoWithoutIndexer() {
    return new LoadCargoCommand(intakeSubsystem, transferSubsystem, indexerSubsystem::isFullSensorTripped, null);
  }

  private Command waitForCargoCount(int count) {
    return (
        new WaitUntilCommand(() -> indexerSubsystem.getCargoCount() >= count || indexerSubsystem.isFullSensorTripped())
          .deadlineWith(loadCargoWithIndexer()));
  }

  private Command turnToAngle(double angle) {
    return new TurnToAngleCommand(angle, driveTrainSubsystem);
  }

}
