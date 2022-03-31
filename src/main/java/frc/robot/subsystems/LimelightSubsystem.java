package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {
  //valid keys - https://docs.limelightvision.io/en/latest/networktables_api.html
  private final static String ntTargetValid = "tv";
  private final static String ntTargetX = "tx";
  private final static String ntTargetY = "ty";

  private final NetworkTable limelightNetworkTable;
  private final String networkTableName;

  private boolean targetValid = false;
  private long targetLastSeen = 0;
  private double targetX = 0;
  private double targetY = 0;

  private final HashMap<String, MedianFilter> updateFilterMap = new HashMap<>();
  
  private boolean enabled;
  private Profile activeProfile = Profile.NEAR;

  public LimelightSubsystem(String networkTableName) {
    
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    this.networkTableName = networkTableName;

    //this adds listeners on an explicit list
    addLimelightUpdateListeners(limelightNetworkTable, ntTargetValid, ntTargetX, ntTargetY);
  }

  private void addLimelightUpdateListeners(NetworkTable limelightTable, String... keys) {
    for (String key : keys) {
      limelightNetworkTable.addEntryListener(key, this::update, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      updateFilterMap.putIfAbsent(key, new MedianFilter(20));
    }
  }

  public ShuffleboardLayout addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Target", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 2)).withPosition(0, 0);
    detailDashboard.addBoolean("Acquired", this::getTargetAcquired).withPosition(0, 0);
    detailDashboard.addNumber("X", this::getTargetX).withPosition(0, 1);
    detailDashboard.addNumber("Y", this::getTargetY).withPosition(1, 1);
    
    dashboard.add(this).withPosition(0, 1);
    return detailDashboard;
  }

  private void update(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {

    switch(key) {

      case ntTargetX:
        targetX = value.getDouble();
        break;

      case ntTargetY:
        targetY = value.getDouble();
        break;

      case ntTargetValid:
        targetValid = value.getDouble() == 1.0;
        break;
    }
  }

  @Override
  public void periodic() {
    // Flush NetworkTable to send LED mode and pipeline updates immediately
    var shouldFlush = (limelightNetworkTable.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0) || 
        limelightNetworkTable.getEntry("pipeline").getDouble(0.0) != activeProfile.pipelineId);
  
    limelightNetworkTable.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    limelightNetworkTable.getEntry("camMode").setDouble(enabled ? 0.0 : 0.0);
    limelightNetworkTable.getEntry("pipeline").setDouble(activeProfile.pipelineId);
  
    if (shouldFlush)  {
      NetworkTableInstance.getDefault().flush();
    }
  }

  public boolean getTargetAcquired() {
    return targetValid;
  }

  public long getTargetLastSeen() {
    return targetLastSeen;
  }

  public double getTargetX() {
    return targetX;
  }

  public double getTargetY() {
    return targetY;
  }

  /**
   * Turns the LEDS off and switches camera mode to driver.
   */
  public void disable() {
    enabled = false;
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode
   * to vision processor.
   */
  public void enable() {
    enabled = true;
  }

  public void setProfile(final Profile profile) {
    activeProfile = profile;
  }

  public Profile getProfile() {
    return activeProfile;
  }

  public String getNetworkTableName() {
    return networkTableName;
  }

}