package frc.robot.subsystems;

/**
 * Enum of values for the Jetson CargoColor filter. This is used with the JetsonSubsystem
 */
public enum CargoColor {
  /** Only target Red Cargo */
  Red("RedCargo"),
  /** Only target Blue Cargo */
  Blue("BlueCargo"),
  /** Target Red and Blue Cargo */
  Both("Both");

  private String color;

  private CargoColor(String color) {
    this.color = color;
  }

  /** Gets the String representation of the color that is sent to the Jetson
   * @return string value to sent to the Jetson
   */
  public String getJetsonString() {
    return color;
  }
}
