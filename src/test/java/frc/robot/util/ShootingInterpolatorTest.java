package frc.robot.util;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;

import org.junit.Before;
import org.junit.Test;

public class ShootingInterpolatorTest {

  /** shooting interpolator under test */
  private ShootingInerpolator shootingInterpolator;

  @Before
  public void setUp() {
    // Create a new interpolator for each test. This way the known values are not carried over between tests.
    shootingInterpolator = new ShootingInerpolator();
  }

  /**
   * Test getting the speed for a distance that is a known value.
   */
  @Test
  public void testDirectMatch() {
    shootingInterpolator.put(2d, 5d);
    shootingInterpolator.put(3d, 6d);

    assertThat(shootingInterpolator.interpolate(2d), equalTo(5d));
  }

  /**
   * Test getting the speed for a distance that's easily calculated.
   */
  @Test
  public void testSimple() {
    shootingInterpolator.put(4d, 14d);
    shootingInterpolator.put(6d, 16d);

    assertThat(shootingInterpolator.interpolate(5d), equalTo(15d));
  }

  /**
   * Test getting the speed for a distance when the values are whole integers.
   */
  @Test
  public void testInteger() {
    shootingInterpolator.put(4d, 14d);
    shootingInterpolator.put(6d, 16d);

    assertThat(shootingInterpolator.interpolate(5d), equalTo(15d));
  }

  /**
   * Test getting the speed for a distance when the values are decimals.
   */
  @Test
  public void testDecimal() {
    shootingInterpolator.put(4.75, 14.75);
    shootingInterpolator.put(6.75, 16.75);

    assertThat(shootingInterpolator.interpolate(5.25), equalTo(15.25));
  }

  /**
   * Test getting the speed for a distance that is larger than the largest known value.
   */
  @Test
  public void testAbove() {
    shootingInterpolator.put(2d, 5d);
    shootingInterpolator.put(3d, 10d);

    assertThat(shootingInterpolator.interpolate(5d), equalTo(10d));
  }

  /**
   * Test getting the speed for a distance that is smaller than the smallest known value.
   */
  @Test
  public void testBelow() {
    shootingInterpolator.put(2d, 5d);
    shootingInterpolator.put(3d, 10d);

    assertThat(shootingInterpolator.interpolate(1d), equalTo(5d));
  }

}
