package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;
import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_INTAKE_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_SPACER_SENSOR;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MultiplexedColorSensor;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);
  private final RelativeEncoder indexerEncoder;
  private final SparkMaxPIDController pidController;
  
  private final MultiplexedColorSensor intakeColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_INTAKE_SENSOR);
  private final MultiplexedColorSensor spacerColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_SPACER_SENSOR);
  private final MultiplexedColorSensor fullColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_FULL_SENSOR);
  
  // Proximity thresholds for when to trip each sensor
  private static final int THRESHOLD_INTAKE = 240;
  private static final int THRESHOLD_SPACE = 400;
  private static final int THRESHOLD_FULL = 300;

  private boolean shooting;
  private int ballCount = 0;

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
  private final Color kRed = new Color(1,0,0);
  private final Color kBlue = new Color(0,0,1);

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setIdleMode(IdleMode.kCoast);
    indexer.setOpenLoopRampRate(0.1);
    indexerEncoder = indexer.getAlternateEncoder(Type.kQuadrature, 8192);
    pidController = indexer.getPIDController();
    pidController.setFeedbackDevice(indexerEncoder);
    indexer.burnFlash();
    updateColorSensors();
    colorMatch.addColorMatch(kRed);
    colorMatch.addColorMatch(kBlue);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 2)).withPosition(0, 0);
    detailDashboard.addNumber("Intake Prox", () -> intakeProximity).withPosition(0, 0);
    detailDashboard.addNumber("Spacer Prox", () -> spacerProximity).withPosition(1, 0);
    detailDashboard.addNumber("Full Prox", () -> fullProximity).withPosition(0, 1);
    
    dashboard.add(this).withPosition(0, 1);
  }

  public void addDriverDashboardWidgets(ShuffleboardTab dashboard) {
    var colorSensorLayout = dashboard.getLayout("Indexer Color", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 2))
        .withSize(2, 2).withPosition(5, 0);
    intakeColorWidget = colorSensorLayout.addBoolean("Intake", () -> true).withPosition(0, 0);
    spacerColorWidget = colorSensorLayout.addBoolean("Spacer", () -> true).withPosition(1, 0);
    fullColorWidget = colorSensorLayout.addBoolean("Full", () -> true).withPosition(0, 1);
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

  public boolean isReadyForBall() {
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
    if (!isIntakeSensorTripped()){
      intakeColorString = "Black";
    } else if(colorMatchResult.color == kRed){
      intakeColorString = "Red";
    } else if(colorMatchResult.color == kBlue){
      intakeColorString = "Blue";
    }
    return intakeColorString;
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