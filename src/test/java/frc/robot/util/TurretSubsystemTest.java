package frc.robot.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystemTest {
  
  /**
   * Tests the Potentiometer offset constant calculation by testing the value at 180-degrees (straight backwords)
   */
  @Test
  public void testPotOffset() {
    // Find what the sensor will read at 180 degrees
    var actualSensorReadAt180 = ((TurretConstants.SOFT_LIMIT_FORWARD + TurretConstants.SOFT_LIMIT_REVERSE) /2d);
    var actualSensorDegreesAt180 = actualSensorReadAt180 * TurretConstants.POTENTIOMETER_COEFFICIENT;

    // Assert that applying the offset results in 180-degrees
    assertEquals(180, actualSensorDegreesAt180 + TurretConstants.POTENTIOMETER_OFFSET, 0);
  }

}
