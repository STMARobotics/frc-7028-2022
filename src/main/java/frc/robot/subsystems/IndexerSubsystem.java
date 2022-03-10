package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;
import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_INTAKE_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_SPACER_SENSOR;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MultiplexedColorSensor;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);
  private final RelativeEncoder indexerEncoder;
  private final SparkMaxPIDController pidController;
  
  private final MultiplexedColorSensor intakeSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_INTAKE_SENSOR);
  private final MultiplexedColorSensor spacerSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_SPACER_SENSOR);
  private final MultiplexedColorSensor fullSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_FULL_SENSOR);

  // Proximity thresholds for when to trip each sensor
  private static final int THRESHOLD_INTAKE = 240;
  private static final int THRESHOLD_SPACE = 400;
  private static final int THRESHOLD_FULL = 300;

  private boolean shooting;
  private int ballCount = 0;

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setIdleMode(IdleMode.kCoast);
    indexer.setOpenLoopRampRate(0.1);
    indexerEncoder = indexer.getAlternateEncoder(Type.kQuadrature, 8192);
    pidController = indexer.getPIDController();
    pidController.setFeedbackDevice(indexerEncoder);
    indexer.burnFlash();

    //add triggers for ball count management
    // new Trigger(this::isFullSensorTripped).whenInactive(this::onFullSensorCleared);
    // new Trigger(this::isSpacerSensorTripped).whenActive(this::onSpaceSensorTripped).whenInactive(this::onSpaceSensorCleared);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    dashboard.addNumber("Balls", () -> ballCount).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 5));
    // detailDashboard.addBoolean("Intake", this::isIntakeSensorTripped);
    // detailDashboard.addBoolean("Spacer", this::isSpacerSensorTripped);
    // detailDashboard.addBoolean("Full", this::isFullSensorTripped);
  }
  
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Intake Prox", intakeSensor.getProximity());
    // SmartDashboard.putNumber("Spacer Prox", intakeSensor.getProximity());
    // SmartDashboard.putNumber("Full Prox", intakeSensor.getProximity());
  }

  /**
   * Called when full sensor becomes clear
   */
  void onFullSensorCleared() {
    if(getIndexerValue() >= 0) {
      decrementBallCount();
    }
  }

  /**
   * Called when spacer sensor becomes tripped
   */
  void onSpaceSensorTripped() {
    if (getIndexerValue() >= 0) {
      incrementBallCount(); //if we're moving forward then increment count as soon as we hold a ball here
    }
  }

  /**
   * Called when spacer sensor becomes clear
   */
  void onSpaceSensorCleared() {
    if (getIndexerValue() < 0) {
      decrementBallCount(); //if we clear the space sensor moving in reverse we lost a ball
    }
  }

  /**
   * Decrements the ball count
   */
  private void decrementBallCount() {
    ballCount = MathUtil.clamp(ballCount - 1, 0, 2);
  }

  /**
   * Increments the ball count
   */
  private void incrementBallCount() {
    ballCount = MathUtil.clamp(ballCount + 1, 0, 2);
  }

  private double getIndexerValue() {
    return indexer.get();
  }

  public void intake() {
    // sensors return false when something is detected
    if ((!isIntakeSensorTripped() && !isSpacerSensorTripped() && !isFullSensorTripped()) || //if all sensors are clear stop the belt
        (isFullSensorTripped() || (!isSpacerSensorTripped() && !isIntakeSensorTripped()))) {  //if fullsensor is tripped OR intake and 2nd sensor are clear
      stop();
    } else {
      load();
    }
  }

  public void prepareToShoot() {
    if (isFullSensorTripped()) {
      stop();
    } else {
      load();
    }
  }

  public boolean isReadyForBall() {
    return (!isIntakeSensorTripped() && !isSpacerSensorTripped() && !isFullSensorTripped())
        || (shooting && !isIntakeSensorTripped() && !isSpacerSensorTripped());
  }

  public boolean isFullSensorTripped() {
    return fullSensor.getProximity() > THRESHOLD_FULL;
  }

  private boolean isIntakeSensorTripped() {
    return intakeSensor.getProximity() > THRESHOLD_INTAKE;
  }

  public boolean isSpacerSensorTripped() {
    return spacerSensor.getProximity() > THRESHOLD_SPACE;
  }

  public boolean isRunning() {
    return indexer.getOutputCurrent() != 0;
  }

  public int getBallCount() {
    return ballCount;
  }

  public void resetBallCount(int ballCount) {
    this.ballCount = ballCount;
  }

  public void load() {
    indexer.set(.5);
    shooting = false;
  }

  public void unload() {
    indexer.set(-.5);
    shooting = false;
  }

  public void shoot() {
    indexer.set(.7);
    shooting = true;
  }

  public void stop() {
    indexer.set(0);
    shooting = false;
  }

}