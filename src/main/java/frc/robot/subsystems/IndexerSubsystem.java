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
import com.revrobotics.ColorMatchResult;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MultiplexedColorSensor;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);
  private final RelativeEncoder indexerEncoder;
  private final SparkMaxPIDController pidController;
  
  private final MultiplexedColorSensor multiplexer = new MultiplexedColorSensor(Port.kMXP);

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
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    intakeColorWidget = dashboard.addBoolean("Intake Color", () -> true);
    spacerColorWidget = dashboard.addBoolean("Spacer Color", () -> true);
    fullColorWidget = dashboard.addBoolean("Full Color", () -> true);
    // detailDashboard.addBoolean("Intake", this::isIntakeSensorTripped);
    // detailDashboard.addBoolean("Spacer", this::isSpacerSensorTripped);
    // detailDashboard.addBoolean("Full", this::isFullSensorTripped);
    // detailDashboard.addNumber("Intake Prox", intakeSensor::getProximity);
    // detailDashboard.addNumber("Spacer Prox", intakeSensor::getProximity);
    // detailDashboard.addNumber("Full Prox", intakeSensor::getProximity);
  }
  
  private void updateColorSensors(){
    multiplexer.setChannel(PORT_ID_INTAKE_SENSOR);
    intakeColor = multiplexer.getColor();
    intakeProximity = multiplexer.getProximity();
    multiplexer.setChannel(PORT_ID_SPACER_SENSOR);
    spacerColor = multiplexer.getColor();
    spacerProximity = multiplexer.getProximity();
    multiplexer.setChannel(PORT_ID_FULL_SENSOR);
    fullColor = multiplexer.getColor();
    fullProximity = multiplexer.getProximity();
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
    updateColorSensors();

    ColorMatchResult intakeResult = colorMatch.matchClosestColor(intakeColor);
    String intakeColorString = "Black";
    if (!isIntakeSensorTripped()){
      intakeColorString = "Black";
    } else if(intakeResult.color == kRed){
      intakeColorString = "Red";
    } else if(intakeResult.color == kBlue){
      intakeColorString = "Blue";
    }
    intakeColorWidget.withProperties(Map.of("colorWhenTrue", intakeColorString));

    ColorMatchResult spacerResult = colorMatch.matchClosestColor(spacerColor);
    String spacerColorString = "Black";
    if(!isSpacerSensorTripped()){
      spacerColorString = "Black";
    } else if(spacerResult.color == kRed){
      spacerColorString = "Red";
    } else if(spacerResult.color == kBlue){
      spacerColorString = "Blue";
    }
    spacerColorWidget.withProperties(Map.of("colorWhenTrue", spacerColorString));

    ColorMatchResult fullResult = colorMatch.matchClosestColor(fullColor);
    String fullColorString = "Black";
    if(!isFullSensorTripped()){
      fullColorString = "Black";
    } else if(fullResult.color == kRed){
      fullColorString = "Red";
    } else if(fullResult.color == kBlue){
      fullColorString = "Blue";
    }
    fullColorWidget.withProperties(Map.of("colorWhenTrue", fullColorString));
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