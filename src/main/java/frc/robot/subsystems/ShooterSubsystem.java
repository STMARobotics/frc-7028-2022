// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.CLOSED_LOOP_ERROR_RANGE;
import static frc.robot.Constants.ShooterConstants.COUNTS_PER_REVOLUTION;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_FOLLOWER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_LEADER;
import static frc.robot.Constants.ShooterConstants.RAMP_RATE;
import static frc.robot.Constants.ShooterConstants.SHOOTING_INTERPOLATOR;
import static frc.robot.Constants.ShooterConstants.kA;
import static frc.robot.Constants.ShooterConstants.kP;
import static frc.robot.Constants.ShooterConstants.kS;
import static frc.robot.Constants.ShooterConstants.kV;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX leader = new WPI_TalonFX(DEVICE_ID_SHOOTER_LEADER);
  private final WPI_TalonFX follower = new WPI_TalonFX(DEVICE_ID_SHOOTER_FOLLOWER);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS,kV, kA);

  private double targetSpeed = 0;

  public ShooterSubsystem() {
    leader.configFactoryDefault();
    follower.configFactoryDefault();
    
    var config = new TalonFXConfiguration();
    config.slot0.kP = kP;
    config.slot0.kI = 0;
    config.slot0.kD = 0;
    config.closedloopRamp = RAMP_RATE;
    leader.configAllSettings(config);
    follower.configAllSettings(config);
    
    leader.enableVoltageCompensation(true);
    follower.enableVoltageCompensation(true);
    leader.setNeutralMode(NeutralMode.Coast);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.setInverted(true);
    follower.follow(leader);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    dashboard.addNumber("Velocity", leader::getSelectedSensorVelocity).withWidget(BuiltInWidgets.kGraph);
    dashboard.addNumber("Target Velocity", leader::getClosedLoopTarget);
    dashboard.addNumber("Error", leader::getClosedLoopError).withWidget(BuiltInWidgets.kGraph);
  }

  /**
   * Runs the shooter at the specified velocity
   * @param speed velocity set point
   */
  public void runShooter(double speed) {
    targetSpeed = speed;
    leader.set(
      ControlMode.Velocity, 
      speed,
      DemandType.ArbitraryFeedForward, 
      feedforward.calculate(speed, leader.getSelectedSensorVelocity() - speed) / COUNTS_PER_REVOLUTION);
  }

  public void prepareToShoot(double distance) {
    var speed = SHOOTING_INTERPOLATOR.interpolate(distance);
    runShooter(speed);
  }

  public boolean isReadyToShoot() {
    return Math.abs(leader.getSelectedSensorVelocity() - targetSpeed) <= CLOSED_LOOP_ERROR_RANGE;
  }

  public void stop() {
    leader.set(ControlMode.PercentOutput, 0);
  }

}
