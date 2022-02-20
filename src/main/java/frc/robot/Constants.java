/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveTrainConstants {
    public static final int DEVICE_ID_RIGHT_LEADER = 0;
    public static final int DEVICE_ID_RIGHT_FOLLOWER = 1;
    public static final int DEVICE_ID_LEFT_LEADER = 2;
    public static final int DEVICE_ID_LEFT_FOLLOWER = 3;

    public static final int EDGES_PER_ROTATION = 2048;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = 0.7101550117116572;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = 1.01;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 2.93;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = 0.761;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kP = 0.154;

    public static final double CLOSED_LOOP_RAMP = .2;
    public static final double OPEN_LOOP_RAMP = .25;
  }

  public static final class ShooterConstants {
    public static final int DEVICE_ID_SHOOTER_LEADER = 10;
    public static final int DEVICE_ID_SHOOTER_FOLLOWER = 11;
  }

  public static final class IndexerConstants {
    public static final int DEVICE_ID_BELT = 6;
  }

  public static final class IntakeConstants {
    public static final int DEVICE_ID_INTAKE = 7;
  }

  public static final class TransferConstants {
    public static final int DEVICE_ID_TRANSFER_LEADER = 8;
    public static final int DEVICE_ID_TRANSFER_FOLLOWER = 5;
  }

  public static final class ControllerConstants {
    public static final int PORT_ID_DRIVER_CONTROLLER = 0;
    public static final int PORT_ID_OPERATOR_CONSOLE = 1;
  }

  public static final class TrajectoryConstants {

    // Max speed in meters per second
    public static final double MAX_SPEED_AUTO = 3;

    // Max acceleration in meters per second per second
    public static final double MAX_ACCELERATION_AUTO = 2;

    // Max voltage
    public static final double MAX_VOLTAGE_AUTO = 11;

    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
        FEED_FORWARD, DRIVE_KINEMATICS, MAX_VOLTAGE_AUTO);

    // Baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static final class DriverConstants {

    public static final double ROTATION_MULTIPLIER = 1;

    public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
    public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

    public static final double DEADBAND_HIGH = 0.12;
    public static final double DEADBAND_LOW = -DEADBAND_HIGH;
  }

  public static final class ArcadeConstants {
    // Max speed to drive in teleop in meters per second
    public static final double MAX_SPEED_ARCADE = 3.7;

    // Max angular velocity in teleop in radians per second
    public static final double MAX_ANGULAR_VEL_ARCADE = Units.degreesToRadians(360);

    // Max rate of change for speed per second
    public static final double SPEED_RATE_LIMIT_ARCADE = 2.5;

    // Max rate of change for rotation per second
    public static final double ROTATE_RATE_LIMIT_ARCADE = 3.0;
  }
}