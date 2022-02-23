package frc.robot.util;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;

import java.util.Map;

import org.junit.Test;

public class ShootingInterpolatorTest {

  /**
   * Test getting the speed for a distance that is a known value.
   */
  @Test
  public void testDirectMatch() {
    ShootingInterpolator shootingInterpolator =
        new ShootingInterpolator(Map.ofEntries(Map.entry(2d, 5d), Map.entry(3d, 6d)));

    assertThat(shootingInterpolator.interpolate(2d), equalTo(5d));
  }

  /**
   * Test getting the speed for a distance when the values are whole integers.
   */
  @Test
  public void testInteger() {
    ShootingInterpolator shootingInterpolator =
        new ShootingInterpolator(Map.ofEntries(Map.entry(4d, 14d), Map.entry(6d, 16d)));

    assertThat(shootingInterpolator.interpolate(5d), equalTo(15d));
  }

  /**
   * Test getting the speed for a distance when the values are decimals.
   */
  @Test
  public void testDecimal() {
    ShootingInterpolator shootingInterpolator =
        new ShootingInterpolator(Map.ofEntries(Map.entry(4.75d, 14.75d), Map.entry(6.75d, 16.75d)));
    
    assertThat(shootingInterpolator.interpolate(5.25), equalTo(15.25));
  }

  /**
   * Test getting the speed for a distance that is larger than the largest known value.
   */
  @Test
  public void testAbove() {
    ShootingInterpolator shootingInterpolator =
        new ShootingInterpolator(Map.ofEntries(Map.entry(2d, 5d), Map.entry(3d, 10d)));

    assertThat(shootingInterpolator.interpolate(5d), equalTo(10d));
  }

  /**
   * Test getting the speed for a distance that is smaller than the smallest known value.
   */
  @Test
  public void testBelow() {
    ShootingInterpolator shootingInterpolator =
        new ShootingInterpolator(Map.ofEntries(Map.entry(2d, 5d), Map.entry(3d, 10d)));

    assertThat(shootingInterpolator.interpolate(1d), equalTo(5d));
  }

}
