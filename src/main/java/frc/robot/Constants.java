/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;

import java.util.Map;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.util.ShootingInterpolator;

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
    public static final int DEVICE_ID_LEFT_LEADER = 0;
    public static final int DEVICE_ID_LEFT_FOLLOWER = 1;
    public static final int DEVICE_ID_RIGHT_LEADER = 2;
    public static final int DEVICE_ID_RIGHT_FOLLOWER = 3;

    public static final int EDGES_PER_ROTATION = 2048;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 625d / 72d;

    public static final double TRACK_WIDTH_METERS = 0.7101550117116572;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = 0.57854;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 1.9308;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = 0.51442;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    // Values from SysID angular analysis used by simulation
    public static final double kV_ANGULAR = 1.5;
    public static final double kA_ANGULAR = 0.3;

    public static final double kP = 0;

    public static final double CLOSED_LOOP_RAMP = .2;
    public static final double OPEN_LOOP_RAMP = .25;
  }

  public static final class ShooterConstants {
    public static final int DEVICE_ID_SHOOTER_LEADER = 10;
    public static final int DEVICE_ID_SHOOTER_FOLLOWER = 11;

    public static final double kS = 0.52864;
    public static final double kV = 0.10689;
    public static final double kA = 0.0064378;

    public static final double kP = 0.0020087;

    public static final int EDGES_PER_REVOLUTION = 2048;

    public static final double RAMP_RATE = 0.4;
    public static final int CLOSED_LOOP_ERROR_RANGE = 200;

    public static final ShootingInterpolator SHOOTING_INTERPOLATOR = new ShootingInterpolator(Map.ofEntries(
      Map.entry(Units.inchesToMeters(55.95), 11000d),
      Map.entry(Units.inchesToMeters(69.33), 11700d),
      Map.entry(Units.inchesToMeters(84.90), 12200d),
      Map.entry(Units.inchesToMeters(127.02), 13400d),
      Map.entry(Units.inchesToMeters(134.51), 13900d),
      Map.entry(Units.inchesToMeters(168.33), 14800d),
      Map.entry(Units.inchesToMeters(188.35), 15300d)));
  }

  public static final class TurretConstants {
    public static final int DEVICE_ID_TURRET = 5;
    public static final int DEVICE_ID_PIGEON = 12;

    public static final double kP_PIGEON = 1d;
    public static final double kD_PIGEON = 0d;

    public static final double kP_POTENTIOMETER = 0d;
    public static final double kD_POTENTIOMETER = 0d;

    /** Forward soft limit in native sensor units */
    public static final int SOFT_LIMIT_FORWARD = -130;
    /** Reverse soft limit in native sensor units */
    public static final int SOFT_LIMIT_REVERSE = -391;
    
    /** Range of motion between soft limits in degrees */
    public static final double RANGE_OF_MOTION = 183;

    /** Value to multiple by the native sensor value to get degrees */
    public static final double POTENTIOMETER_COEFFICIENT = 
        TurretConstants.RANGE_OF_MOTION / (TurretConstants.SOFT_LIMIT_FORWARD - TurretConstants.SOFT_LIMIT_REVERSE);
    
    /** Forward soft limit in degrees (after sensor coefficient is applied) */
    public static final double SOFT_LIMIT_FORWARD_DEGREES = SOFT_LIMIT_FORWARD * POTENTIOMETER_COEFFICIENT;

    /** Reverse soft limit in degrees (after sensor coefficient is applied) */
    public static final double SOFT_LIMIT_REVERSE_DEGREES = SOFT_LIMIT_REVERSE * POTENTIOMETER_COEFFICIENT;

    public static final double CLOSED_LOOP_MAX_OUTPUT = 0.7;

    /** Pigeon native units per revolution */
    public static final double PIGEON_UNITS_PER_REVOLUTION = 8192d;
  }

  public static final class IndexerConstants {
    public static final int DEVICE_ID_INDEXER = 6;

    public static final int PORT_ID_INTAKE_SENSOR = 3;
    public static final int PORT_ID_SPACER_SENSOR = 4;
    public static final int PORT_ID_FULL_SENSOR = 2;

    public static final double BELT_kP = 0;
    public static final double BELT_RUN_SPEED = 11000;
  }

  public static final class IntakeConstants {
    public static final int DEVICE_ID_INTAKE = 7;
    
    public static final int CHANNEL_SOLENOID_FORWARD = 1;
    public static final int CHANNEL_SOLENOID_BACKWARD = 2;
  }

  public static final class TransferConstants {
    public static final int DEVICE_ID_TRANSFER = 8;
  }

  public static final class ControllerConstants {
    public static final int PORT_ID_DRIVER_CONTROLLER = 0;
    public static final int PORT_ID_OPERATOR_CONSOLE = 1;
  }
  
  public static final class LimeLightConstants {

    public static final double PIPELINE_INDEX_NEAR = 0.0;
    public static final double PIPELINE_INDEX_FAR = 1.0;

    /** Height of the target in meters */
    public static final double TARGET_HEIGHT = Units.inchesToMeters(104);

    public static final LimelightConfig LIMELIGHT_CONFIG = LimelightConfig.Builder.create()
        .withMountDepth(Units.inchesToMeters(9))
        .withMountDistanceFromCenter(Units.inchesToMeters(0))
        .withMountingAngle(43)
        .withMountingHeight(Units.inchesToMeters(35.875))
        .withNetworkTableName("limelight")
        .build();
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

    public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
    public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

    public static final double DEADBAND_HIGH = 0.12;
    public static final double DEADBAND_LOW = -DEADBAND_HIGH;

    // Max rate of change for speed per second (1 / <value> is the number of seconds allowed to go from 0 to max)
    public static final double SPEED_RATE_LIMIT_ARCADE = 2.25;

    // Max rate of change for rotation per second (1 / <value> is the number of seconds allowed to go from 0 to max)
    public static final double ROTATE_RATE_LIMIT_ARCADE = 5.0;
  }

  public static final class AimConstants {

    public static final double kP = 0.023;
    public static final double kD = 0.0;
    public static final double AIM_TOLERANCE = 1.75;

    public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
    public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

    public static final double DEADBAND_HIGH = 0.12;
    public static final double DEADBAND_LOW = -DEADBAND_HIGH;
  }

  public static final class ArcadeConstants {
    // Max speed to drive in teleop in meters per second
    public static final double MAX_SPEED_ARCADE = 3.7;

    // Max angular velocity in teleop in radians per second
    public static final double MAX_ANGULAR_VEL_ARCADE = Units.degreesToRadians(270);
  }
}