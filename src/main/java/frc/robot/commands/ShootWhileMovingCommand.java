package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootWhileMovingCommand extends CommandBase {
  
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chasisSpeedsSupplier;
  private final DoubleSupplier targetAngleProvider;

  public ShootWhileMovingCommand(
      ShooterSubsystem shooterSubsystem,
      IndexerSubsystem indexerSubsystem, 
      TurretSubsystem turretSubsystem, 
      ShooterLimelightSubsystem limelightSubsystem,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chasisSpeedsSupplier,
      DoubleSupplier targetAngleProvider) {

    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.poseSupplier = poseSupplier;
    this.chasisSpeedsSupplier = chasisSpeedsSupplier;
    this.targetAngleProvider = targetAngleProvider;

    addRequirements(shooterSubsystem, indexerSubsystem, turretSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
  }

  @Override
  public void execute() {
    
    if (limelightSubsystem.getTargetAcquired()) {
      // get the robot pose and speed
      var chasisSpeeds = chasisSpeedsSupplier.get();
      var robotPose = poseSupplier.get();
  
      // calculate the field-relative X and Y velocity
      var xVelocity = chasisSpeeds.vxMetersPerSecond * robotPose.getRotation().getCos();
      var yVelocity = chasisSpeeds.vxMetersPerSecond * robotPose.getRotation().getSin();

      // target distance and turret angle setpoints
      var targetDistance = limelightSubsystem.getDistanceToTarget();
      var targetAngle = turretSubsystem.getAngleToRobot() - limelightSubsystem.getTargetX();

      // TODO update the targetDistance and turretAngle variables to account for robot moving


      shooterSubsystem.prepareToShoot(targetDistance);
      var atTarget = Math.abs(turretSubsystem.getAngleToRobot() - targetAngle) < AimConstants.AIM_TOLERANCE;
      if (atTarget && shooterSubsystem.isReadyToShoot()) {
        indexerSubsystem.shoot();
      } else {
        indexerSubsystem.prepareToShoot();
      }
    } else {
      // try to find the target
      turretSubsystem.positionToRobotAngle(targetAngleProvider.getAsDouble());
    }

  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    indexerSubsystem.stop();
    turretSubsystem.stop();
  }

}
