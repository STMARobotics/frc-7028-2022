package frc.robot.util;

import java.util.TreeMap;

/**
 * Map of numeric keys to numeric values that can interpolate between values.
 */
public class InterpolatingMap {

    private final TreeMap<Double, Double> treeMap = new TreeMap<>();

    /**
     * Puts a known value into the map.
     * @param key key
     * @param value value
     */
    public void put(double key, double value) {
        treeMap.put(key, value);
    }

    /**
     * Returns or interpolates a value for the given key. If the key is in the map, the value is returned.
     * If the key is not in the map, the value is interpolated from the surrounding values.
     * If the key is larger than the largest key in the map, the largest key value is returned. SImilarly, if the key is
     * smaller than the smallest key in the map, the smallest key value is returned.
     * @param key key to look up or interpolate
     * @return the value for the key, either as stored or interpolated
     */
    public double interpolate(double key) {
        var exactMatch = treeMap.get(key);
        if (exactMatch != null) {
            // The key is in the map, so just return the value
            return exactMatch;
        }

        // Get surrounding keys for interpolation
        var topBound = treeMap.ceilingEntry(key);
        var bottomBound = treeMap.floorEntry(key);

        // If attempting interpolation at ends of tree, return the nearest data point
        if (topBound == null && bottomBound == null) {
            return 0;
        } else if (topBound == null) {
            return bottomBound.getValue();
        } else if (bottomBound == null) {
            return topBound.getValue();
        }

        // Inerpolate the value
        var low_key = bottomBound.getKey();
        var high_key = topBound.getKey();
        var low_value = bottomBound.getValue();
        var high_value = topBound.getValue();

        return (low_value + (key - low_key) * (high_value - low_value) / (high_key - low_key));
    }
    
}
