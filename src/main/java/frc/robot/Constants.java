// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivetrainConstants {
    public static final int DEVICE_ID_LEFT_LEADER = 0;
    public static final int DEVICE_ID_LEFT_FOLLOWER = 1;
    public static final int DEVICE_ID_RIGHT_LEADER = 2;
    public static final int DEVICE_ID_RIGHT_FOLLOWER = 3;
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
}
