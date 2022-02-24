package frc.robot;

/**
 * A filter that can handle a deadband, useful for joysticks that don't center at exactly 0.
 */
public class DeadbandFilter {

  private double low;
  private double high;

  /**
   * Constructor. Values above the low low and below high limit are filtered to zero.
   * @param low  low limit
   * @param high high limit
   */
  public DeadbandFilter(double low, double high) {
    this.low = low;
    this.high = high;
  }

  /**
   * Sets the high limit
   * @param high values above this value and below the low are filtered to zero
   */
  public void setHigh(double high) {
    this.high = high;
  }

  /**
   * Gets the high limit
   * @return the high limit
   */
  public double getHigh() {
    return high;
  }

  /**
   * Sets the low limit
   * @param zoneLow values below this value and above the high are filtered to zero
   */
  public void setZoneLow(double low) {
    this.low = low;
  }

  /**
   * Gets the low limit
   * @return the low limit
   */
  public double getZoneLow() {
    return low;
  }

  /**
   * Applies the filter to a value
   * @param value value to filter
   * @return zero if the given value is within the deadband, otherwise the original value
   */
  public double calculate(double value) {
    if (value > this.low && value < this.high) {
      return 0;
    }
    return value;
  }

}