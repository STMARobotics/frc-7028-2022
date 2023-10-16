package frc.robot.util;

import static frc.robot.subsystems.DriveTrainSubsystem.edgesPerDecisecToMetersPerSec;
import static frc.robot.subsystems.DriveTrainSubsystem.edgesToMeters;
import static frc.robot.subsystems.DriveTrainSubsystem.metersPerSecToEdgesPerDecisec;
import static frc.robot.subsystems.DriveTrainSubsystem.metersToEdges;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Tests for the drivetrain unit conversion functions.
 */
public class DriveTrainSubsystemTest {

  /**
   * Tests converting from edges to meters and back
   */
  @Test
  public void testInvertConvertEdges() {
    assertEquals(2048d, metersToEdges(edgesToMeters(2048d)), 0.001d);
    ;
  }

  /**
   * Tests converting from meters to edges and back
   */
  @Test
  public void testInvertConvertMeters() {
    assertEquals(2048d, edgesToMeters(metersToEdges(2048d)), 0.001d);
    ;
  }

  /**
   * Tests convertion from edges per decisecond to meters per second and back
   */
  @Test
  public void testInvertConvertEdgesPerDeci() {
    assertEquals(2048d, metersPerSecToEdgesPerDecisec(edgesPerDecisecToMetersPerSec(2048d)), 0.001d);
  }

  /**
   * Tests convertion from meters per second to edges per decisecond and back
   */
  @Test
  public void testInvertConvertMetersPerSec() {
    assertEquals(2048d, edgesPerDecisecToMetersPerSec(metersPerSecToEdgesPerDecisec(2048d)), 0.001d);
  }

}
