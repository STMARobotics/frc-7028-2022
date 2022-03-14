package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that tracks the position of the robot and attempts to keep the turret pointed in the direction of the hub.
 */
public class TrackTargetCommand extends CommandBase {

  // The hub is in the center of the field. The field is 54' x 27'
  private static final Pose2d hubPose = 
      new Pose2d(Units.inchesToMeters(54 * 12) / 2, Units.inchesToMeters(27 * 12) / 2, new Rotation2d());

  private final Supplier<Pose2d> poseSupplier;
  private double angleToTarget;
  private double distanceToTarget;

  /**
   * Constructs a TrackTargetCommand
   * @param poseSupplier supplier to use to get the robot's current pose
   */
  public TrackTargetCommand(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void execute() {
    // For all measurements, the grid is on the field with the X axis running the long way across the field, with the
    // the blue alliance on the lesser X side, and the red alliance on the greater X side. The Y axis runs the shot
    // way across the field. CCW rotation is positive.

    // The robot's pose is on a coordinate grid with (0,0) at the bottom left corner.
    var robotCurrentPose = poseSupplier.get();

    // Get the robot's pose relative to the target. This will be the pose of the robot on a grid with the hub in the
    // center at (0,0)
    var relativePose = robotCurrentPose.relativeTo(hubPose);

    // Get the angle to the hub from the robot's position
    var angleToHub = Math.atan(relativePose.getY() / relativePose.getX());

    // Calculate the direction to turn the turret, considering the direction the robot's drivetrain is in
    var headingSetpoint = Units.radiansToDegrees(angleToHub) - robotCurrentPose.getRotation().getDegrees();
    
    // Deal with quadrants where X is > 0 (the robot is to the right of the hub)
    if (relativePose.getX() > 0) {
      headingSetpoint += 180;
    }
    
    // Deal with quadrants that result in a negative or out of range value
    headingSetpoint = (headingSetpoint + 360) % 360;

    angleToTarget = headingSetpoint;
    distanceToTarget = Math.sqrt(Math.pow(relativePose.getY(), 2) + Math.pow(relativePose.getX(), 2));
  }

  public void addDriverDashboardWidgets(ShuffleboardTab driverTab) {
    driverTab.addBoolean("Target Distance", () -> distanceToTarget > Units.inchesToMeters(48)).withPosition(5, 2);
    driverTab.addBoolean("Target Reachable", () -> TurretSubsystem.isInRange(angleToTarget)).withPosition(6, 2);
  }

  /**
   * Gets the angle to the target, relative to the robot. Range is [0,360], 0 is robot front, CCW is positive.
   * @return angle to the target
   */
  public double getAngleToTarget() {
    return angleToTarget;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
  
}
