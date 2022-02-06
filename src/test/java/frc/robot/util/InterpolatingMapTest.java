package frc.robot.util;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;

import org.junit.Before;
import org.junit.Test;

public class InterpolatingMapTest {

    private InterpolatingMap interpolatingMap;

    @Before
    public void setUp() {
        interpolatingMap = new InterpolatingMap();
    }

    @Test
    public void testDirectMatch() {
        interpolatingMap.put(2d, 5d);
        interpolatingMap.put(3d, 6d);

        assertThat(interpolatingMap.interpolate(2d), equalTo(5d));
    }
    
    @Test
    public void testSimple() {
        interpolatingMap.put(4d, 14d);
        interpolatingMap.put(6d, 16d);
        
        assertThat(interpolatingMap.interpolate(5d), equalTo(15d));
    }

    @Test
    public void testInteger() {
        interpolatingMap.put(4d, 14d);
        interpolatingMap.put(6d, 16d);
        
        assertThat(interpolatingMap.interpolate(5d), equalTo(15d));
    }

    @Test
    public void testDecimal() {
        interpolatingMap.put(4.75, 14.75);
        interpolatingMap.put(6.75, 16.75);

        assertThat(interpolatingMap.interpolate(5.25), equalTo(15.25));
    }

}
