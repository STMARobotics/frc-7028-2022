// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftLeader = new WPI_TalonFX(LEFT_LEADER_ID);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(LEFT_FOLLOWER_ID);
  private final WPI_TalonFX rightLeader = new WPI_TalonFX(RIGHT_LEADER_ID);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(RIGHT_FOLLOWER_ID);
  private final DifferentialDrive m_Drive = new DifferentialDrive(leftLeader, rightLeader);
  

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {

    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
