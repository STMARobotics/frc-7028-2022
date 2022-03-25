// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.CLOSED_LOOP_ERROR_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_FOLLOWER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_LEADER;
import static frc.robot.Constants.ShooterConstants.EDGES_PER_REVOLUTION;
import static frc.robot.Constants.ShooterConstants.RAMP_RATE;
import static frc.robot.Constants.ShooterConstants.SHOOTING_INTERPOLATOR;
import static frc.robot.Constants.ShooterConstants.kA;
import static frc.robot.Constants.ShooterConstants.kP;
import static frc.robot.Constants.ShooterConstants.kS;
import static frc.robot.Constants.ShooterConstants.kV;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX leader = new WPI_TalonFX(DEVICE_ID_SHOOTER_LEADER);
  private final WPI_TalonFX follower = new WPI_TalonFX(DEVICE_ID_SHOOTER_FOLLOWER);

  // Shooter feed forward from SysID. This is configured for rotations per second.
  // Note, CTRE native velocities are in
  // encoder edges per 100 ms.
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);
  private double gain = 0;
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

    leader.enableVoltageCompensation(false);
    follower.enableVoltageCompensation(false);
    leader.setNeutralMode(NeutralMode.Coast);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.setInverted(true);
    follower.follow(leader);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailLayout = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 1)).withPosition(0, 0);
    detailLayout.addNumber("Raw Velocity", leader::getSelectedSensorVelocity);
    detailLayout.addNumber("Velocity RPS", () -> edgesPerDecisecToRPS(leader.getSelectedSensorVelocity()));
    detailLayout.addNumber("Target Velocity",
        () -> leader.getControlMode() == ControlMode.Velocity ? leader.getClosedLoopTarget() : 0);
    detailLayout.addNumber("Error",
        () -> leader.getControlMode() == ControlMode.Velocity ? leader.getClosedLoopError() : 0);
    
    dashboard.add(this).withPosition(0, 1);
  }

  public void addDriverDashboardWidgets(ShuffleboardTab driverTab) {
    // var gainEntry = driverTab.addPersistent("Gain", 0).withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("Max", 60, "Min", -60)).withSize(2, 1).withPosition(9, 1).getEntry();
    // gainEntry.addListener(this::updateGain, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    // gain = gainEntry.getDouble(0);
  }

  // private void updateGain(EntryNotification notification) {
  //   gain = Units.inchesToMeters(notification.value.getDouble());
  // }

  /**
   * Runs the shooter at the specified velocity
   * 
   * @param speed velocity set point in native units (encoder edges per 100 ms)
   */
  public void runShooter(double speed) {
    // Feedforward is configured for rotations per second so a conversion from
    // native units is needed.
    var feedForwardVolts = feedForward.calculate(edgesPerDecisecToRPS(targetSpeed), edgesPerDecisecToRPS(speed), .02);

    // Arbitrary feed forward is a value in the range [-1, 1], which is a percentage
    // of the saturation voltage
    leader.set(
        ControlMode.Velocity,
        speed,
        DemandType.ArbitraryFeedForward,
        feedForwardVolts / RobotController.getBatteryVoltage());

    targetSpeed = speed;
  }

  public void prepareToShoot(double distance) {
    var speed = SHOOTING_INTERPOLATOR.interpolate(distance + gain);
    runShooter(speed);
  }

  /**
   * Converts from edges per decisecond to rotations per second.
   * 
   * @param edgesPerDecisec velocity in edges per decisecond (native Talon units)
   * @return velocity in rotations per second
   */
  public static double edgesPerDecisecToRPS(double edgesPerDecisec) {
    var rotationsPerDecisecond = edgesPerDecisec / EDGES_PER_REVOLUTION;
    return rotationsPerDecisecond * 10;
  }

  /**
   * Converts from rotations per second to edges per decisecond.
   * 
   * @param rps velocity in RPM
   * @return velocity in edges per decisecond (native talon units)
   */
  public static double rpsToedgesPerDecisec(double rps) {
    var edgesPerSecond = rps * EDGES_PER_REVOLUTION;
    return edgesPerSecond / 10;
  }

  public boolean isReadyToShoot() {
    return Math.abs(leader.getSelectedSensorVelocity() - targetSpeed) <= CLOSED_LOOP_ERROR_TOLERANCE;
  }

  public void stop() {
    leader.set(ControlMode.PercentOutput, 0);
  }

}
