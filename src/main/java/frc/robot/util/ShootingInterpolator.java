package frc.robot.util;

import java.util.Collections;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Class that takes a table of known distances from the target and their shooter
 * power settings, and then interpolates values in between the known values.
 */
public class ShootingInterpolator {

  /**
   * Stores a sorted table of known distance and speed values
   */
  private final NavigableMap<Double, Double> knownValuesMap;;

  public ShootingInterpolator(Map<Double, Double> knownValues) {
    this.knownValuesMap = Collections.unmodifiableNavigableMap(new TreeMap<>(knownValues));
  }

  /**
   * Puts a known distance and speed into the table.
   * 
   * @param distance     distance from the target
   * @param shooterSpeed speed of the shooter
   */
  public void put(double distance, double shooterSpeed) {
    knownValuesMap.put(distance, shooterSpeed);
  }

  /**
   * Returns or interpolates a speed for the given distance. If the distance is
   * known, the shooter speed is returned.
   * If the distance is not known, the speed is interpolated from the surrounding
   * speeds.
   * If the distance is larger than the largest known distance, the largest
   * distance speed is returned. Similarly, if the distance is smaller than the
   * smallest known distance, the smallest distance speed value is returned.
   * 
   * @param distance the distance from the target
   * @return the shooter speed for the given distance
   */
  public double interpolate(double distance) {
    var exactMatch = knownValuesMap.get(distance);
    if (exactMatch != null) {
      // The distance is in the map, so just return the speed
      return exactMatch;
    }

    // Get surrounding distances for interpolation
    var higher = knownValuesMap.ceilingEntry(distance);
    var lower = knownValuesMap.floorEntry(distance);

    // If attempting interpolation at ends of tree, return the nearest data point
    if (higher == null && lower == null) {
      return 0;
    } else if (higher == null) {
      return lower.getValue();
    } else if (lower == null) {
      return higher.getValue();
    }

    // Inerpolate the value
    var low_distance = lower.getKey();
    var high_distance = higher.getKey();
    var low_speed = lower.getValue();
    var high_speed = higher.getValue();

    return (low_speed + (distance - low_distance) * (high_speed - low_speed) / (high_distance - low_distance));
  }

}
