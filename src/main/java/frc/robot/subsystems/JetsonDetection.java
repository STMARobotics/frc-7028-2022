package frc.robot.subsystems;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.UpperCamelCaseStrategy;
import com.fasterxml.jackson.databind.annotation.JsonNaming;

/**
 * This class holds the data sent by the Jetson. The Jetson sends one big JSON string of data for a detection instead
 * of sending each value in a separate network table entry so we are can be certain to get all matching values for a
 * frame.
 */
@JsonNaming(UpperCamelCaseStrategy.class)
public class JetsonDetection {
  public String classLabel;
  public String classID;
  public String instanceID;
  public double area;
  public double bottom;
  public double centerX;
  public double centerY;
  public double confidence;
  public double height;
  public double left;
  public double right;
  public double top;
  public double width;
  public double timestamp;
  public double targetX;
  public double targetY;
  public double targetDistance;
}

