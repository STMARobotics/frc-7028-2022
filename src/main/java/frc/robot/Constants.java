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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
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
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(19.65d);
    public static final double DRIVE_GEAR_RATIO = 2700d / 240d;

    public static final double TRACK_WIDTH_METERS = 0.6608;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = 0.70888;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 2.4834;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = 0.29679;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    // Values from SysID angular analysis used by simulation
    public static final double kV_ANGULAR = 1.5;
    public static final double kA_ANGULAR = 0.3;

    public static final double kP = 0.05;

    public static final double CLOSED_LOOP_RAMP = .2;
    public static final double OPEN_LOOP_RAMP = .25;
  }

  public static final class ShooterConstants {
    public static final int DEVICE_ID_SHOOTER_LEADER = 10;
    public static final int DEVICE_ID_SHOOTER_FOLLOWER = 11;

    public static final double kS = 0.52411;
    public static final double kV = 0.104625;
    public static final double kA = 0.0068293;

    public static final double kP = 0.035;

    public static final int EDGES_PER_REVOLUTION = 2048;

    public static final double RAMP_RATE = 0.2;
    public static final int CLOSED_LOOP_ERROR_TOLERANCE = 200;

    public static final double SHOOT_TIME = 0.1;

    public static final ShootingInterpolator SHOOTING_INTERPOLATOR = new ShootingInterpolator(Map.ofEntries(
      Map.entry(Units.inchesToMeters(40d), 11000d),
      Map.entry(Units.inchesToMeters(56d), 11700d),
      Map.entry(Units.inchesToMeters(60d), 12100d),
      Map.entry(Units.inchesToMeters(80d), 12600d),
      Map.entry(Units.inchesToMeters(90d), 12865d),
      Map.entry(Units.inchesToMeters(100d), 13375d),
      Map.entry(Units.inchesToMeters(120d), 14300d),
      Map.entry(Units.inchesToMeters(140d), 15250d),
      Map.entry(Units.inchesToMeters(160d), 16350d),
      Map.entry(Units.inchesToMeters(180d), 17200d),
      Map.entry(Units.inchesToMeters(200d), 18100d)));

    public static final double MAX_DISTANCE = SHOOTING_INTERPOLATOR.getMax();
    public static final double MIN_DISTANCE = SHOOTING_INTERPOLATOR.getMin();
  }

  public static final class TurretConstants {
    public static final int DEVICE_ID_TURRET = 5;
    public static final int DEVICE_ID_PIGEON = 12;

    public static final double kP_PIGEON = 1d;
    public static final double kD_PIGEON = 0d;

    public static final double kP_POTENTIOMETER = 64d;
    public static final double kD_POTENTIOMETER = 0d;

    // Native sensor units are in range of [0,1023], or [-1023,0] when sensor phase is true (inverted)
    /** Forward soft limit in native sensor units */
    public static final int SOFT_LIMIT_FORWARD = -156;
    /** Reverse soft limit in native sensor units */
    public static final int SOFT_LIMIT_REVERSE = -400;
    
    /** Range of motion between soft limits in degrees */
    public static final double RANGE_OF_MOTION = 183;

    /** Value to multiply by the potentiometer native sensor value to get degrees */
    public static final double POTENTIOMETER_COEFFICIENT = 
        TurretConstants.RANGE_OF_MOTION / (TurretConstants.SOFT_LIMIT_REVERSE - TurretConstants.SOFT_LIMIT_FORWARD);

    /**
     *  The offset of the potentiometer reading from the robot angle. Adding this to the absolute degree reading
     *  from the potentiometer will get the direction the turret is pointed relative to the robot drivetrain.
     */
    // Calculate degrees for the reverse limit from the POT. Subtract the reverse limit degrees based on the range of
    // motion, assuming the turret is mounted straight (e.g. pointed straight backwards is 1/2 the full range of motion)
    // This seems complex for a fixed value, but if the way the potentiometer is mounted on the robot changes, only
    // the soft-limit values need to be updated and everything will still work.
    public static final double POTENTIOMETER_OFFSET =
        (360 - (TurretConstants.SOFT_LIMIT_REVERSE * TurretConstants.POTENTIOMETER_COEFFICIENT)) -
        (180 - TurretConstants.RANGE_OF_MOTION / 2);

    public static final double CLOSED_LOOP_MAX_OUTPUT = 0.7;

    /** Pigeon native units per revolution */
    public static final double PIGEON_UNITS_PER_REVOLUTION = 8192d;
  }

  public static final class IndexerConstants {
    public static final int DEVICE_ID_INDEXER = 6;

    public static final int PORT_ID_INTAKE_SENSOR = 3;
    public static final int PORT_ID_SPACER_SENSOR = 4;
    public static final int PORT_ID_FULL_SENSOR = 2;

    // Proximity thresholds for when to trip each sensor
    public static final int THRESHOLD_INTAKE = 240;
    public static final int THRESHOLD_SPACE = 400;
    public static final int THRESHOLD_FULL = 225;

    public static final double BELT_kP = 0d;
    public static final double BELT_RUN_SPEED = 8000;
    public static final double BELT_SHOOT_SPEED = 8000;

    /** Color of red cargo */
    public static final Color COLOR_RED = new Color(0.510009765625, 0.365234375, 0.125);

    /** Color of blue cargo */
    public static final Color COLOR_BLUE = new Color(0.15283203125, 0.407958984375, 0.439453125);

    /** Color of no cargo */
    public static final Color COLOR_NONE = new Color(0.252197265625, 0.491455078125, 0.2568359375);
  }

  public static final class IntakeConstants {
    public static final int DEVICE_ID_INTAKE = 7;
    
    public static final int CHANNEL_SOLENOID_FORWARD = 1;
    public static final int CHANNEL_SOLENOID_BACKWARD = 0;
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
        .withMountDepth(Units.inchesToMeters(11))
        .withMountDistanceFromCenter(Units.inchesToMeters(0))
        .withMountingAngle(34.28)
        .withMountingHeight(Units.inchesToMeters(36.75))
        .withNetworkTableName("limelight")
        .build();
  }
    
  public static final class TrajectoryConstants {

    // Max speed in meters per second
    public static final double MAX_SPEED_AUTO = 3.5;

    // Max acceleration in meters per second per second
    public static final double MAX_ACCELERATION_AUTO = 2.5;

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

    public static final DeadbandFilter DEADBAND_FILTER = new DeadbandFilter(DEADBAND_LOW, DEADBAND_HIGH);

    // Max rate of change for speed per second (1 / <value> is the number of seconds allowed to go from 0 to max)
    public static final double SPEED_RATE_LIMIT_ARCADE = 1.75d;

    // Max rate of change for rotation per second (1 / <value> is the number of seconds allowed to go from 0 to max)
    public static final double ROTATE_RATE_LIMIT_ARCADE = 5d;
  }

  public static final class AimConstants {

    public static final double kP = 0.023;
    public static final double kD = 0.0;
    public static final double AIM_TOLERANCE = 1.5;

    public static final double AIM_ROTATION_SPEED = 1d;

    public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
    public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

    public static final double DEADBAND_HIGH = 0.12;
    public static final double DEADBAND_LOW = -DEADBAND_HIGH;

    /** The radius of the hub upper target ring */
    public static final double HUB_TARGET_RADIUS = Units.inchesToMeters(49.375 / 2d);
  }

  public static final class ArcadeConstants {
    // Max speed to drive in teleop in meters per second
    public static final double MAX_SPEED_ARCADE = 3.7;

    // Max angular velocity in teleop in radians per second
    public static final double MAX_ANGULAR_VEL_ARCADE = Units.degreesToRadians(270);
  }

  public static final class FieldConstants {

    public static final Pose2d hubPose = 
      new Pose2d(Units.inchesToMeters(54 * 12) / 2, Units.inchesToMeters(27 * 12) / 2, new Rotation2d());

  }

  public static final class ClimbConstants {

    public static DeadbandFilter CLIMB_DEADBAND_FILTER = new DeadbandFilter(-.25, .5);

    public static final int DEVICE_ID_FIRST_STAGE_CLIMB = 12;

    public static final double SOFT_LIMIT_FIRST_STAGE_FWD = 430000;
    public static final double SOFT_LIMIT_FIRST_STAGE_REV = 0;
  }
}