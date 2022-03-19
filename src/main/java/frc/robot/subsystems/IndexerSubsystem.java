package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;
import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_INTAKE_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_SPACER_SENSOR;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.MultiplexedColorSensor;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);
  private final SparkMaxPIDController pidController;
  
  private final MultiplexedColorSensor intakeColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_INTAKE_SENSOR);
  private final MultiplexedColorSensor spacerColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_SPACER_SENSOR);
  private final MultiplexedColorSensor fullColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_FULL_SENSOR);
  
  // Proximity thresholds for when to trip each sensor
  private static final int THRESHOLD_INTAKE = 240;
  private static final int THRESHOLD_SPACE = 400;
  private static final int THRESHOLD_FULL = 225;

  private boolean shooting;
  private int cargoCount = 0;

  private Color intakeColor;
  private Color spacerColor;
  private Color fullColor;
  private int intakeProximity;
  private int spacerProximity;
  private int fullProximity;

  private SuppliedValueWidget<Boolean> intakeColorWidget;
  private SuppliedValueWidget<Boolean> spacerColorWidget;
  private SuppliedValueWidget<Boolean> fullColorWidget;

  private ColorMatch colorMatch = new ColorMatch();

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setIdleMode(IdleMode.kCoast);
    indexer.setOpenLoopRampRate(0.1);
    pidController = indexer.getPIDController();
    pidController.setP(0);//IndexerConstants.BELT_kP);
    pidController.setFF(0.00009);
    indexer.burnFlash();
    updateColorSensors();
    colorMatch.addColorMatch(IndexerConstants.COLOR_RED);
    colorMatch.addColorMatch(IndexerConstants.COLOR_BLUE);
    colorMatch.addColorMatch(IndexerConstants.COLOR_NONE);

    //add triggers for cargo count management
    new Trigger(this::isFullSensorTripped).whenInactive(this::fullSensorCleared);
    new Trigger(this::isSpacerSensorTripped).whenActive(this::spaceSensorTripped).whenInactive(this::spaceSensorCleared);
    
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 3)).withPosition(0, 0);
    detailDashboard.addNumber("Intake Prox", () -> intakeProximity).withPosition(0, 0);
    detailDashboard.addNumber("Spacer Prox", () -> spacerProximity).withPosition(1, 0);
    detailDashboard.addNumber("Full Prox", () -> fullProximity).withPosition(0, 1);
    detailDashboard.addNumber("Cargo Count", this::getCargoCount).withPosition(1, 1);
    detailDashboard.addNumber("Velocity", () -> indexer.getEncoder().getVelocity()).withPosition(0, 2);
    detailDashboard.addNumber("Error", () -> IndexerConstants.BELT_RUN_SPEED - indexer.getEncoder().getVelocity())
        .withPosition(1, 2);
    
    dashboard.add(this).withPosition(0, 1);
  }

  public void addDriverDashboardWidgets(ShuffleboardTab dashboard) {
    var colorSensorLayout = dashboard.getLayout("Indexer", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
        .withSize(1, 3).withPosition(5, 0);
    fullColorWidget = colorSensorLayout.addBoolean("Full", () -> true).withPosition(0, 0);
    spacerColorWidget = colorSensorLayout.addBoolean("Spacer", () -> true).withPosition(0, 1);
    intakeColorWidget = colorSensorLayout.addBoolean("Intake", () -> true).withPosition(0, 2);
  }

  
  void fullSensorCleared() {
    if(indexer.getOutputCurrent() >= 0) {
      decrementCargoCount();
    }
  }

  void spaceSensorTripped() {
    if (indexer.getOutputCurrent() >= 0) {
      incrementCargoCount(); //if indexer moving forward then increment count as soon as we hold a cargo here
    }
  }

  void spaceSensorCleared() {
    if (indexer.getOutputCurrent() < 0) {
      decrementCargoCount(); //if cargo clears the space sensor moving in reverse we lost a cargo
    }
  }

  private void decrementCargoCount() {
    cargoCount = MathUtil.clamp(cargoCount - 1, 0, 2);
  }

  private void incrementCargoCount() {
    cargoCount = MathUtil.clamp(cargoCount + 1, 0, 2);
  }

  /**
   * Updates the color sensor values. We do this only once per period to avoid flooding I2C.
   */
  private void updateColorSensors(){
    var intakeValues = intakeColorSensor.getValues();
    intakeColor = intakeValues.color;
    intakeProximity = intakeValues.proximity;
    var spacerValues = spacerColorSensor.getValues();
    spacerColor = spacerValues.color;
    spacerProximity = spacerValues.proximity;
    var fullValues = fullColorSensor.getValues();
    fullColor = fullValues.color;
    fullProximity = fullValues.proximity;
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

  public boolean isReadyForCargo() {
    return (!isIntakeSensorTripped() && !isSpacerSensorTripped() && !isFullSensorTripped())
        || (shooting && !isIntakeSensorTripped() && !isSpacerSensorTripped());
  }

  @Override
  public void periodic() {
    // Read color sensor values
    updateColorSensors();

    // Update the dashboard with color indicators
    intakeColorWidget.withProperties(Map.of("colorWhenTrue", getColorMatchString(intakeColor)));
    spacerColorWidget.withProperties(Map.of("colorWhenTrue", getColorMatchString(spacerColor)));
    fullColorWidget.withProperties(Map.of("colorWhenTrue", getColorMatchString(fullColor)));
  }

  private String getColorMatchString(Color color) {
    var colorMatchResult = colorMatch.matchClosestColor(color);
    String intakeColorString = "Black";
    if (colorMatchResult.color == IndexerConstants.COLOR_NONE){
      intakeColorString = "Black";
    } else if(colorMatchResult.color == IndexerConstants.COLOR_RED){
      intakeColorString = "Red";
    } else if(colorMatchResult.color == IndexerConstants.COLOR_BLUE){
      intakeColorString = "Blue";
    }
    return intakeColorString;
  }

  /**
   * Returns the color detected by the intake sensor if there is something in front of it, otherwise null.
   * @return the color detected or null if nothing in front of the sensor
   */
  public Color getIntakeColor() {
    return colorMatch.matchClosestColor(intakeColor).color;
  }

  /**
   * Returns the color detected by the spacer sensor if there is something in front of it, otherwise null.
   * @return the color detected or null if nothing in front of the sensor
   */
  public Color getSpacerColor() {
     return colorMatch.matchClosestColor(spacerColor).color;
  }

  /**
   * Returns the color detected by the full sensor if there is something in front of it, otherwise null.
   * @return the color detected or null if nothing in front of the sensor
   */
  public Color getFullColor() {
    return colorMatch.matchClosestColor(fullColor).color;
  }

  public boolean isFullSensorTripped() {
    return fullProximity > THRESHOLD_FULL;
  }

  private boolean isIntakeSensorTripped() {
    return intakeProximity > THRESHOLD_INTAKE;
  }

  public boolean isSpacerSensorTripped() {
    return spacerProximity > THRESHOLD_SPACE;
  }

  public boolean isRunning() {
    return indexer.getOutputCurrent() != 0;
  }

  public int getCargoCount() {
    return cargoCount;
  }

  public void resetCargoCount(int cargoCount) {
    this.cargoCount = cargoCount;
  }

  public void load() {
    pidController.setReference(IndexerConstants.BELT_RUN_SPEED, ControlType.kVelocity);
    shooting = false;
  }

  public void unload() {
    indexer.set(-.5);
    shooting = false;
  }

  public void shoot() {
    pidController.setReference(IndexerConstants.BELT_SHOOT_SPEED, ControlType.kVelocity);
    shooting = true;
  }

  public void stop() {
    pidController.setReference(0, ControlType.kVelocity);
    shooting = false;
  }

}