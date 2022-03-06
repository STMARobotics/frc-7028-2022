package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that tracks the position of the robot and attempts to keep the turret pointed in the direction of the hub.
 */
public class TrackTargetCommand extends CommandBase {

  // The hub is in the center of the field. The field is 54' x 27'
  private final Pose2d hubPose = 
      new Pose2d(Units.inchesToMeters(54 * 12) / 2, Units.inchesToMeters(27 * 12) / 2, new Rotation2d());

  private final Supplier<Pose2d> poseSupplier;

  /**
   * Constructs a TrackTargetCommand
   * @param poseSupplier supplier to use to get the robot's current pose
   */
  public TrackTargetCommand(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void execute() {
    // The robot's pose is on a coordinate grid with (0,0) at the bottom left corner.
    var currentPose = poseSupplier.get();

    // Get the robot's pose relative to the target. This will be the pose of the robot on a grid with the hub in the
    // center at (0,0)
    var relativePose = currentPose.relativeTo(hubPose);

    // Use atan2 to get the angle (in radians) from the robot to the hub
    double angleToHub = Math.atan2(relativePose.getY(), relativePose.getX());

    // TODO do any needed translation and then set the turret heading

    SmartDashboard.putNumber("Angle To Hub", Units.radiansToDegrees(angleToHub));

  }
  
}
