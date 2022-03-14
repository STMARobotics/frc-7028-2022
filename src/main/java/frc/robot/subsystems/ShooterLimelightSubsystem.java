package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.LimeLightConstants;

/**
 * Extends LimelightSubsystem to add information about the target distance.
 */
public class ShooterLimelightSubsystem extends LimelightSubsystem {
  public final LimelightConfig limelightConfig;

  public ShooterLimelightSubsystem(LimelightConfig limelightConfig) {
    super(limelightConfig.getNetworkTableName());
    this.limelightConfig = limelightConfig;
  }

  @Override
  public ShuffleboardLayout addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = super.addDashboardWidgets(dashboard);
    detailDashboard.addNumber("Distance", () -> Units.metersToInches(getDistanceToTarget())).withPosition(0, 0);
    return detailDashboard;
  }

  /**
   * Gets the distance from the Limelight to the target. If no target is acquired this will return zero. This value is
   * the distance straight ahead from the limelight to the target, (parallel to the floor.) Use
   * {@link #getDistanceToTarget()} to get the distance from the robot to the target.
   * 
   * @return distance from the Limelight to the target
   */
  private double getLimelightDistanceToTarget() {
    if (getTargetAcquired()) {
      return (LimeLightConstants.TARGET_HEIGHT - limelightConfig.getMountHeight())
          / Math.tan(Units.degreesToRadians(limelightConfig.getMountAngle() + getTargetY()));
    }
    return 0.0;
  }

  /**
  * Gets the distance from the center of the robot front bumper to the base of the target. If no target is acquired this
  * will return zero. This method accounts for the limelight being mounted off-center and behind the front bumper.
  * 
  * @return distance from the center of the robot front bumper to the base of the target
  */
  public double getDistanceToTarget() {
    return Math.sqrt(Math.pow(getLimelightDistanceToTarget(), 2)
        + Math.pow(limelightConfig.getMountDistanceFromCenter(), 2)) - limelightConfig.getMountDepth();
  }

}