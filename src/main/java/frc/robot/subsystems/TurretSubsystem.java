package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Subsystem for the turret.
 */
public class TurretSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(DEVICE_ID_TURRET);
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(TurretConstants.DEVICE_ID_PIGEON);

  public TurretSubsystem() {
    // Pigeon is mounted vertical. See user guide "Custom Mounting Orientation"
    pigeon.configFactoryDefault();
    var pigeonConfig = new Pigeon2Configuration();
    pigeonConfig.MountPosePitch = 90;
    pigeon.configAllSettings(pigeonConfig);

    turretMotor.configFactoryDefault();
    var talonConfig = new TalonSRXConfiguration();
    talonConfig.openloopRamp = 0.2;
    
    // Potentiometer is primary PID to get soft limit support
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
    talonConfig.forwardSoftLimitThreshold = TurretConstants.SOFT_LIMIT_FORWARD;
    talonConfig.forwardSoftLimitEnable = true;
    talonConfig.reverseSoftLimitThreshold = TurretConstants.SOFT_LIMIT_REVERSE;
    talonConfig.reverseSoftLimitEnable = true;
    
    // Pigeon is on aux PID
    talonConfig.slot1.kP = TurretConstants.kP_PIGEON;
    talonConfig.slot1.kD = TurretConstants.kD_PIGEON;
    talonConfig.slot1.closedLoopPeakOutput =  TurretConstants.CLOSED_LOOP_MAX_OUTPUT;
    talonConfig.remoteFilter1.remoteSensorDeviceID = pigeon.getDeviceID();
    talonConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
    talonConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;
    // We don't use Talon's sensor coefficient feature to convert native units to degrees mainly because it lowers
    // precision since the value has to result in an integer. For example, if we use a coefficient of 0.0439 to convert
    // pigeon units to degrees we only get 360 units per revolution vs. the native 8192.

    turretMotor.configAllSettings(talonConfig);
    turretMotor.selectProfileSlot(0, 0);
    turretMotor.selectProfileSlot(1, 1);
    turretMotor.configVoltageCompSaturation(12);
    turretMotor.enableVoltageCompensation(true);
    turretMotor.overrideLimitSwitchesEnable(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setSensorPhase(false);
    turretMotor.setInverted(true);
    // TODO should we also use current limiting in case we DO hit the hard stop?
    turretMotor.setSafetyEnabled(true);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    dashboard.addNumber("Gyro Angle", this::getAngle);
    dashboard.addNumber("Gyro Heading", this::getHeading);
    dashboard.addNumber("Angle To Robot", this::getRobotRelativeAngle);
    dashboard.addNumber("Native Position", turretMotor::getSelectedSensorPosition);
    dashboard.addNumber("Setpoint",
        () -> turretMotor.getControlMode() == ControlMode.Position ? turretMotor.getClosedLoopTarget() : Double.NaN);
    dashboard.addNumber("Error",
        () -> turretMotor.getControlMode() == ControlMode.Position ? turretMotor.getClosedLoopError() : Double.NaN);
  }

  public void run(double speed) {
    SmartDashboard.putNumber("Turret Power", speed);
    turretMotor.set(speed);
  }

  /**
   * Positions the turret to the specified angle and holds it. NOTE: The angle is the
   * continuous angle (continues from 360 to 361) so the setpoint needs to account for
   * the current angle or you may be attempting to turn the turret more than 360-degrees.
   * @param angle angle to set and hold
   */
  public void positionToAngleWithGyro(double angle) {
    turretMotor.config_kP(0, 0);
    turretMotor.config_kD(0, 0);
    turretMotor.set(TalonSRXControlMode.Position, 0, DemandType.AuxPID, degreesToNativePigeonUnits(angle));
  }

  /**
   * Positions the turret to the specified angle relative to the robot in range of [0,360], 0 is forward CCW+
   * @param angle angle in degrees
   */
  public void positionToRobotAngle(double angle) {
    turretMotor.config_kP(0, TurretConstants.kP_POTENTIOMETER);
    turretMotor.config_kD(0, TurretConstants.kD_POTENTIOMETER);

    // Get angle with zero as straight backward, CW+
    var angleWithCenterZero = 180 - angle;
    // Feed forward is the sine of the angle, times our feed-forwad constant
    var feedForward = Math.sin(Units.degreesToRadians(angleWithCenterZero)) * TurretConstants.FEED_FORWARD;
    turretMotor.set(
        TalonSRXControlMode.Position, 
        degreesPositionToNativePot(angle),
        DemandType.ArbitraryFeedForward,
        feedForward);
  }

  /**
   * Sets the current heading to the specified value. This is used to tell the turret
   * what it's current heading is, not to move the turret to a heading.
   * @param heading new heading value
   */
  public void resetHeading(double heading) {
    // Stop first, in case we're currently trying to hold a heading since the reference is changing
    stop();
    pigeon.setYaw(-heading);
  }

  /**
   * Gets the current heading compatible with Pose2d and Rotation2d. This value is not continous. CW is positive.
   * @return current heading in range of [-180, 180]
   */
  public double getHeading() {
    return Math.IEEEremainder(pigeon.getAngle(), 360.0d);
  }

  /**
   * Gets the current angle heading. This value IS continous (continues from 360 to 361). CCW is positive.
   * @return the current heading
   */
  public double getAngle() {
    return pigeon.getYaw();
  }

  /**
   * Gets the angle of the turret from its position on the robot. CCW is positive.
   * For reference:
   * <ul>
   * <li>90 pointed left</li>
   * <li>180 pointed straight back</li>
   * <li>270 pointed right</li>
   * </ul>
   * @return angle of the turret in relation to the robot
   */
  public double getRobotRelativeAngle() {
    return nativePotPositionToDegrees(turretMotor.getSelectedSensorPosition());
  }

  /**
   * Resets the heading of the turret based on the heading of the robot.
   * @param robotHeading heading of the robot (drivetrain)
   */
  public void resetHeadingToRobot(double robotHeading) {
    resetHeading(robotHeading + getRobotRelativeAngle());
  }

  public void stop() {
    turretMotor.set(0);
  }

  /**
   * Converts a native potentiometer position to a position in degrees
   * @param pot potentiometer position
   * @return position in degrees
   */
  static double nativePotPositionToDegrees(double pot) {
    return pot * TurretConstants.POTENTIOMETER_COEFFICIENT + TurretConstants.POTENTIOMETER_OFFSET;
  }

  /**
   * Converts a position in degrees to a native potentiometer position
   * @param degrees position in degrees
   * @return potentiometer
   */
  static double degreesPositionToNativePot(double degrees) {
    return (degrees - TurretConstants.POTENTIOMETER_OFFSET) / TurretConstants.POTENTIOMETER_COEFFICIENT;
  }

  /**
   * Converts from native pigeon units to degrees
   * @param pigeon pigeon reading
   * @return degree reading
   */
  static double nativePigeonUnitsToDegrees(double pigeon) {
    // Negate since WPI angle is cw+ but Pigeon Yaw is ccw+
    return pigeon * (-360d / TurretConstants.PIGEON_UNITS_PER_REVOLUTION);
  }

  /**
   * Convers from degrees to native pigeon units
   * @param degrees degree reading
   * @return pigeon reading
   */
  static double degreesToNativePigeonUnits(double degrees) {
    // Negate since WPI angle is cw+ but Pigeon Yaw is ccw+
    return degrees / (-360d / TurretConstants.PIGEON_UNITS_PER_REVOLUTION);
  }

}
