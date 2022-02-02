// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX leader = new WPI_TalonFX(0);
  private final WPI_TalonFX follower = new WPI_TalonFX(1);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.57831, 0.09430, 0);// 0.01044

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    leader.configFactoryDefault();
    leader.setSafetyEnabled(true);
    follower.configFactoryDefault();
    follower.setSafetyEnabled(true);

    var config = new TalonFXConfiguration();
    config.slot0.kP = 0.13;
    config.slot0.kI = 0;
    config.slot0.kD = 0;
    config.closedloopRamp = 0.4;
    leader.configAllSettings(config);

    leader.setNeutralMode(NeutralMode.Coast);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.setInverted(true);
    follower.follow(leader);
  }

  @Override
  public void periodic() {
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
    leader.set(0);
  }

}
