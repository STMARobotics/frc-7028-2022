package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.InstantWhenDisabledCommand;

/**
 * Subsystem for interacting with the NVidia Jetson for Cargo object detection. This class uses NetworkTables to
 * exchange data with the Jetson.
 */
public class JetsonSubsystem extends SubsystemBase {

  private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("JetsonDetect");
  private SendableChooser<CargoColor> cargoColorChooser = new SendableChooser<>();

  private ObjectMapper objectMapper = new ObjectMapper();
  
  private JetsonDetection closestDetection;
  private boolean enabled = false;
  private boolean isDirty = true;

  public JetsonSubsystem() {
    addJetsonUpdateListener(networkTable, this::updateClosest, "Closest Detection");

    // We only need to run detection when the robot is enabled.
    new Trigger(RobotState::isEnabled)
      .whenActive(this::enable)
      .whenInactive(new InstantWhenDisabledCommand(this::disable));
  
    // Put a chooser on the dashboard for picking which cargo to target
    cargoColorChooser.setDefaultOption("Both", CargoColor.Both);
    cargoColorChooser.addOption("RedCargo", CargoColor.Red);
    cargoColorChooser.addOption("BlueCargo", CargoColor.Blue);
    SmartDashboard.putData("Cargo Color", cargoColorChooser);
  }

  @Override
  public void periodic() {
    // Update the network table with values we want to send
    networkTable.getEntry("Enabled").setBoolean(enabled);
    CargoColor selectedColor = cargoColorChooser.getSelected();
    var cargoColorEntry = networkTable.getEntry("CargoColor");
    isDirty = isDirty || !cargoColorEntry.getString(CargoColor.Both.getJetsonString()).equals(selectedColor.getJetsonString());
    cargoColorEntry.setString(selectedColor.getJetsonString());
    if (isDirty) {
      // Flushing is not required, but if the values are written immediately we will see improved response time
      NetworkTableInstance.getDefault().flush();
      isDirty = false;
    }
  }

  private void addJetsonUpdateListener(NetworkTable limelightTable, TableEntryListener listener, String... keys) {
    for (String key : keys) {
      networkTable.addEntryListener(key, listener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  private void updateClosest(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {
    try {
      var detectionJson = value.getString();
      if (detectionJson == null || detectionJson.isEmpty()) {
        closestDetection = null;
      } else {
        closestDetection = objectMapper.readValue(value.getString(), JetsonDetection.class);
      }
    } catch (Exception e) {
      closestDetection = null;
    }
  }

  /**
   * Get the target object detected closest to the crosshairs
   * @return all of the information about the target object closest to the crosshairs
   */
  public JetsonDetection getClosestDetection() {
    return closestDetection;
  }

  /**
   * Enable object detection
   */
  public void enable() {
    enabled = true;
    isDirty = true;
  }

  /**
   * Disable object detection
   */
  public void disable() {
    enabled = false;
    isDirty = true;
  }
}
