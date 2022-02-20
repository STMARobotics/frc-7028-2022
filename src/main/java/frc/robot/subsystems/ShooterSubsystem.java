// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_FOLLOWER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_LEADER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX leader = new WPI_TalonFX(DEVICE_ID_SHOOTER_LEADER);
  private final WPI_TalonFX follower = new WPI_TalonFX(DEVICE_ID_SHOOTER_FOLLOWER);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.57831, 0.09430, 0);

  public ShooterSubsystem() {
    leader.configFactoryDefault();
    leader.setSafetyEnabled(true);
    follower.configFactoryDefault();

    var config = new TalonFXConfiguration();
    config.slot0.kP = 0.13;
    config.slot0.kI = 0;
    config.slot0.kD = 0;
    config.closedloopRamp = 0.4;
    leader.configAllSettings(config);
    follower.configAllSettings(config);

    leader.setNeutralMode(NeutralMode.Coast);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.setInverted(true);
    follower.follow(leader);
  }

  /**
   * Runs the shooter at the specified velocity
   * @param speed velocity set point
   */
  public void runShooter(double speed) {
    leader.set(
      ControlMode.Velocity, 
      speed,
      DemandType.ArbitraryFeedForward, 
      feedforward.calculate(speed, leader.getSelectedSensorVelocity() - speed) / 2048);
  }

  public void stop() {
    leader.set(ControlMode.PercentOutput, 0);
  }

}
