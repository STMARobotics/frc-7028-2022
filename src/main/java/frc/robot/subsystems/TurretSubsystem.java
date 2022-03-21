package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET;
import static frc.robot.Constants.TurretConstants.SOFT_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.SOFT_LIMIT_REVERSE;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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
    talonConfig.forwardSoftLimitThreshold = SOFT_LIMIT_FORWARD;
    talonConfig.forwardSoftLimitEnable = true;
    talonConfig.reverseSoftLimitThreshold = SOFT_LIMIT_REVERSE;
    talonConfig.reverseSoftLimitEnable = true;
    talonConfig.slot0.kP = TurretConstants.kP_POTENTIOMETER;
    talonConfig.slot0.kD = TurretConstants.kD_POTENTIOMETER;
    
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
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 2)).withPosition(0, 0);

    detailDashboard.addNumber("Angle To Robot", this::getAngleToRobot).withPosition(0, 0);
    detailDashboard.addNumber("Native Position", turretMotor::getSelectedSensorPosition).withPosition(1, 0);
    detailDashboard.addNumber("Setpoint",
        () -> turretMotor.getControlMode() == ControlMode.Position ? turretMotor.getClosedLoopTarget() : Double.NaN)
        .withPosition(0, 1);
    detailDashboard.addNumber("Error",
        () -> turretMotor.getControlMode() == ControlMode.Position ? turretMotor.getClosedLoopError() : Double.NaN)
        .withPosition(1, 1);

    dashboard.add(this).withPosition(0, 1);
  }

  /**
   * Runs the turret motor at the set percentage output
   * @param speed speed to set [-1,1]
   */
  public void run(double speed) {
    turretMotor.set(speed);
  }

  /**
   * Positions the turret to the specified angle relative to the robot in range of [0,360], 0 is forward CCW+
   * @param angle angle in degrees
   */
  public void positionToRobotAngle(double angle) {
    var position = degreesPositionToNativePot(angle);
    turretMotor.set(
        TalonSRXControlMode.Position, 
        MathUtil.clamp(position, TurretConstants.SOFT_LIMIT_REVERSE + 1, SOFT_LIMIT_FORWARD - 1));
  }

  /**
   * Returns true if the specified angle is within the range of the turret for {@link #positionToRobotAngle(double)}
   * @param angle angle to check
   * @return true if in range, false if out of range
   */
  public static boolean isInRange(double angle) {
    var nativePosition = degreesPositionToNativePot(angle);
    return nativePosition < SOFT_LIMIT_FORWARD && nativePosition > SOFT_LIMIT_REVERSE;
  }

  public boolean isClearOfClimb() {
    return getAngleToRobot() > 160 && getAngleToRobot() < 200;
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
  public double getAngleToRobot() {
    return nativePotPositionToDegrees(turretMotor.getSelectedSensorPosition());
  }

  /**
   * Turns off the turret motor
   */
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
