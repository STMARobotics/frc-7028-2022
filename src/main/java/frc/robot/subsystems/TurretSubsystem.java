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
    
    // Potentiometer is primary PID to get soft limit support
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
    // convert native units to degrees
    talonConfig.primaryPID.selectedFeedbackCoefficient = TurretConstants.POTENTIOMETER_COEFFICIENT;
    talonConfig.forwardSoftLimitThreshold = TurretConstants.SOFT_LIMIT_FORWARD_DEGREES;
    talonConfig.forwardSoftLimitEnable = true;
    talonConfig.reverseSoftLimitThreshold = TurretConstants.SOFT_LIMIT_REVERSE_DEGREES;
    talonConfig.reverseSoftLimitEnable = true;
    
    // Pigeon is on aux PID
    talonConfig.slot1.kP = TurretConstants.kP_PIGEON;
    talonConfig.slot1.kD = TurretConstants.kD_PIGEON;
    talonConfig.slot1.closedLoopPeakOutput =  TurretConstants.CLOSED_LOOP_MAX_OUTPUT;
    talonConfig.remoteFilter1.remoteSensorDeviceID = pigeon.getDeviceID();
    talonConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
    // convert native units to degrees
    talonConfig.auxiliaryPID.selectedFeedbackCoefficient = 360d / TurretConstants.PIGEON_UNITS_PER_REVOLUTION;
    talonConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;
    
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
    dashboard.addNumber("Angle", this::getAngle);
    dashboard.addNumber("Heading", this::getHeading);
    dashboard.addNumber("Angle To Robot", this::getRobotRelativeAngle);
    dashboard.addNumber("Setpoint",
        () -> turretMotor.getControlMode() == ControlMode.Position ? turretMotor.getClosedLoopTarget() : Double.NaN);
  }

  public void run(double speed) {
    turretMotor.set(speed);
  }

  /**
   * Positions the turret to the specified angle and holds it. NOTE: The angle is the
   * continuous angle (continues from 360 to 361) so the setpoint needs to account for
   * the current angle or you may be attempting to turn the turret more than 360-degrees.
   * @param angle angle to set and hold
   */
  public void positionToAngle(double angle) {
    // Negate since WPI angle is cw+ but Pigeon Yaw is ccw+
    turretMotor.set(TalonSRXControlMode.Position, 0, DemandType.AuxPID, -angle);
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
   * Gets the current heading. This value is not continous.
   * @return current heading in range of [-180, 180]
   */
  public double getHeading() {
    return Math.IEEEremainder(pigeon.getAngle(), 360.0d);
  }

  /**
   * Gets the current angle heading. This value IS continous (continues from 360 to 361)
   * @return the current heading (may be greater than 360)
   */
  public double getAngle() {
    return pigeon.getAngle();
  }

  /**
   * Gets the angle of the turret from its position on the robot.
   * @return
   */
  public double getRobotRelativeAngle() {
    // TODO this needs to account for the angle of the turret on the robot, it points backwards and
    // the turret's zero will be on one side
    return turretMotor.getSelectedSensorPosition();
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

}
