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
  private WPI_TalonFX motor = new WPI_TalonFX(0);
  private WPI_TalonFX motor1 = new WPI_TalonFX(1);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.53095, 0.0882, 0.0030098);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    motor.configFactoryDefault();
    var config = new TalonFXConfiguration();
    config.slot0.kP = 0.02;
    config.slot0.kI = 0;
    config.slot0.kD = 0;

    motor.configAllSettings(config);

    motor.setNeutralMode(NeutralMode.Coast);
    motor1.setNeutralMode(NeutralMode.Coast);

    motor.setInverted(true);

    motor1.follow(motor);
  }

  @Override
  public void periodic() {
  }

  /**
   * Runs the shooter at the specified velocity
   * @param speed velocity set point
   */
  public void runShooter(double speed) {
    motor.set(
      ControlMode.Velocity, 
      speed,
      DemandType.ArbitraryFeedForward, 
      feedforward.calculate(speed, motor.getSelectedSensorVelocity() - speed) / 2048);
  }

  public void stop() {
    motor.set(0);
  }

}
