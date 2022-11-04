package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.InstantWhenDisabledCommand;

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

  private boolean takeSnapshot = false;

  private boolean targetValid = false;
  private long targetLastSeen = 0;
  private double targetX = 0;
  private double targetY = 0;

  private final HashMap<String, MedianFilter> updateFilterMap = new HashMap<>();
  
  private boolean enabled;
  private boolean driverMode;
  private Profile activeProfile = Profile.NEAR;

  public LimelightSubsystem(String networkTableName) {
    
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    this.networkTableName = networkTableName;

    //this adds listeners on an explicit list
    addLimelightUpdateListeners(limelightNetworkTable, ntTargetValid, ntTargetX, ntTargetY);

    limelightNetworkTable.getEntry("snapshot").setDouble(0.0);

    new Trigger(RobotState::isEnabled)
        .onTrue(new InstantCommand(this::enable))
        .onFalse(new InstantWhenDisabledCommand(this::disable, this));
  }

  private void addLimelightUpdateListeners(NetworkTable limelightTable, String... keys) {
    for (String key : keys) {
      var topic = limelightTable.getTopic(key);
      NetworkTableInstance.getDefault().addListener(topic, NetworkTableEvent.kValueLocal, this::update);
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

  private void update(NetworkTableEvent event) {
    var value = event.valueData.value;
    switch(event.valueData.getTopic().getInfo().name) {

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
    limelightNetworkTable.getEntry("camMode").setDouble(driverMode ? 1.0 : 0.0);
    limelightNetworkTable.getEntry("pipeline").setDouble(activeProfile.pipelineId);
  
    if (shouldFlush)  {
      NetworkTableInstance.getDefault().flush();
    }

    if(takeSnapshot) {
      limelightNetworkTable.getEntry("snapshot").setDouble(1.0);
      takeSnapshot = false;
    } else {
      limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
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
   * Turns the LEDS off and switches the camera mode to vision processor.
   */
  public void disable() {
    enabled = false;
    driverMode = false;
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode
   * to vision processor.
   */
  public void enable() {
    enabled = true;
    driverMode = false;
  }

  /**
   * Sets the LEDs to off and switches the camera to driver mode.
   */
  public void driverMode() {
    enabled = false;
    driverMode = true;
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

  public void takeSnapshot() {
    takeSnapshot = true;
  }

}