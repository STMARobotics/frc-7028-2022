package frc.robot.commands;


import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_INTAKE_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_SPACER_SENSOR;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.util.ColorSensorValues;
import frc.robot.util.MultiplexedColorSensor;

/**
 * Runnable to use with the WPILib Notifier to update the color sensors in the background.
 * This class enables color sensors to be read on a background thread to avoid overrunning
 * the main robot loop.
 */
public class ColorSensorReader implements Runnable {
  
  private final MultiplexedColorSensor intakeColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_INTAKE_SENSOR);
  private final MultiplexedColorSensor spacerColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_SPACER_SENSOR);
  private final MultiplexedColorSensor fullColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_FULL_SENSOR);
  
  private AtomicReference<ColorSensorValues> intakeValues;
  private AtomicReference<ColorSensorValues> spacerValues;
  private AtomicReference<ColorSensorValues> fullValues;

   /**
   * Updates the color sensor values
   */
  public void run(){
    intakeValues.set(intakeColorSensor.getValues());
    spacerValues.set(spacerColorSensor.getValues());
    fullValues.set(fullColorSensor.getValues());
  }

  public ColorSensorValues getIntakeValues() {
    return intakeValues.get();
  }

  public ColorSensorValues getSpacerValues() {
    return spacerValues.get();
  }

  public ColorSensorValues getFullValues() {
    return fullValues.get();
  }

}
