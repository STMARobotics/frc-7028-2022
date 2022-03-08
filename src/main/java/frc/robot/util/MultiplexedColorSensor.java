package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This is a wrapper for the REV Color Sensor V3, allowing multiple ones to be
 * used on a multiplexer board (this was tested using the Adafruit TCA9548A).
 * 
 * <p>
 * This helper class was originally developed by Team 4776. <b>Go S.C.O.T.S. Bots!</b>
 * </p>
 */
public class MultiplexedColorSensor {
  // Static map of multiplexers by I2C port. This is static because it is shared by all color sensors
  // connected to the multiplexer
  private static Map<Port, I2C> multiplexers = new HashMap<>();
  
  // Change this if your multiplexer has a different address. This is the TCA9548A's default address.
  private static final int kMultiplexerAddress = 0x70;

  // I2C for the multiplexer
  private final I2C multiplexer;

  // The actual sensor. All of the methods call this sensor to get the data.
  private ColorSensorV3 sensor;
  
  // What port on the multiplexer the color sensor is plugged into.
  private final int port;

  /**
   * Create a multiplexed color sensor.
   * 
   * @param i2cPort - What port the multiplexer is plugged into.
   * @param port    - What port the color sensor is plugged into the multiplexer
   *                (commonly labeled SC3 and SD3 on the PCB, where 3 is the
   *                port)
   */
  public MultiplexedColorSensor(I2C.Port i2cPort, int port) {
    if (multiplexers.get(i2cPort) == null) {
      multiplexer = new I2C(i2cPort, kMultiplexerAddress);
      multiplexers.put(i2cPort, multiplexer);
    } else {
      multiplexer = multiplexers.get(i2cPort);
    }
    this.port = port;
    setChannel();
    sensor = new ColorSensorV3(i2cPort);
  }

  /**
   * Sets the multiplexer to the correct port before using the color sensor.
   */
  private void setChannel() {
    multiplexer.write(kMultiplexerAddress, 1 << port);
  }

  /*-----------------------------------------------------------------------*/
  /* Below are all of the methods used for the color sensor. */
  /* All this does is set the channel, then run the command on the sensor. */
  /*-----------------------------------------------------------------------*/

  public Color getColor() {
    setChannel();
    return sensor.getColor();
  }

  public int getProximity() {
    setChannel();
    return sensor.getProximity();
  }

  public RawColor getRawColor() {
    setChannel();
    return sensor.getRawColor();
  }

  public int getRed() {
    setChannel();
    return sensor.getRed();
  }

  public int getGreen() {
    setChannel();
    return sensor.getGreen();
  }

  public int getBlue() {
    setChannel();
    return sensor.getBlue();
  }

  public int getIR() {
    setChannel();
    return sensor.getIR();
  }

  public boolean hasReset() {
    setChannel();
    return sensor.hasReset();
  }
}