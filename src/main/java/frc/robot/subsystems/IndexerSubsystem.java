package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.COLOR_BLUE;
import static frc.robot.Constants.IndexerConstants.COLOR_NONE;
import static frc.robot.Constants.IndexerConstants.COLOR_RED;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;
import static frc.robot.Constants.IndexerConstants.THRESHOLD_INTAKE;
import static frc.robot.Constants.IndexerConstants.THRESHOLD_SPACE;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.ColorSensorReader;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;
  
  private final ColorSensorReader colorSensorReader = new ColorSensorReader();
  private final Notifier colorSensorNotifier = new Notifier(colorSensorReader);
  private final Debouncer fullSensorDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  private int cargoCount = 0;

  private SuppliedValueWidget<Boolean> intakeColorWidget;
  private SuppliedValueWidget<Boolean> spacerColorWidget;
  private SuppliedValueWidget<Boolean> fullColorWidget;

  private ColorMatch colorMatch = new ColorMatch();

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setIdleMode(IdleMode.kCoast);
    indexer.setOpenLoopRampRate(0.1);
    indexer.setClosedLoopRampRate(0.1);
    indexer.setInverted(true);
    pidController = indexer.getPIDController();
    pidController.setP(IndexerConstants.BELT_kP);
    pidController.setFF(0.00009);
    encoder = indexer.getEncoder();
    indexer.burnFlash();
    
    colorSensorReader.run();
    // Update the color sensors in the background to prevent loop overrun
    colorSensorNotifier.setName("Color Sensors");
    colorSensorNotifier.startPeriodic(0.02);

    colorMatch.addColorMatch(COLOR_RED);
    colorMatch.addColorMatch(COLOR_BLUE);
    colorMatch.addColorMatch(COLOR_NONE);

    // Add triggers for cargo count management
    new Trigger(this::isFullSensorTripped).whenInactive(this::fullSensorCleared);
    new Trigger(this::isSpacerSensorTripped).whenActive(this::spaceSensorTripped).whenInactive(this::spaceSensorCleared);
    
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 3)).withPosition(0, 0);
    detailDashboard.addNumber("Intake Prox", () -> colorSensorReader.getIntakeValues().proximity).withPosition(0, 0);
    detailDashboard.addNumber("Spacer Prox", () -> colorSensorReader.getSpacerValues().proximity).withPosition(1, 0);
    detailDashboard.addNumber("Full Prox", () -> colorSensorReader.getFullValues().proximity).withPosition(0, 1);
    detailDashboard.addNumber("Cargo Count", this::getCargoCount).withPosition(1, 1);
    
    dashboard.add(this).withPosition(0, 1);
  }

  public void addDriverDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.addNumber("Cargo Count", this::getCargoCount).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0, "Max", 2)).withSize(1, 1).withPosition(8, 3);
    var colorSensorLayout = dashboard.getLayout("Indexer", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
        .withSize(1, 3).withPosition(8, 0);
    fullColorWidget = colorSensorLayout.addBoolean("Full", () -> true).withPosition(0, 0);
    spacerColorWidget = colorSensorLayout.addBoolean("Spacer", () -> true).withPosition(0, 1);
    intakeColorWidget = colorSensorLayout.addBoolean("Intake", () -> true).withPosition(0, 2);
  }

  
  void fullSensorCleared() {
    if(encoder.getVelocity() >= 0) {
      decrementCargoCount();
    }
  }

  void spaceSensorTripped() {
    if (encoder.getVelocity() >= 0) {
      incrementCargoCount(); //if indexer moving forward then increment count as soon as we hold a cargo here
    }
  }

  void spaceSensorCleared() {
    if (encoder.getVelocity() < 0) {
      decrementCargoCount(); //if cargo clears the space sensor moving in reverse we lost a cargo
    }
  }

  private void decrementCargoCount() {
    cargoCount = MathUtil.clamp(cargoCount - 1, 0, 2);
  }

  private void incrementCargoCount() {
    cargoCount = MathUtil.clamp(cargoCount + 1, 0, 2);
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

  @Override
  public void periodic() {
    // Update the dashboard with color indicators
    intakeColorWidget.withProperties(
        Map.of("colorWhenTrue", getColorMatchString(colorSensorReader.getIntakeValues().color)));
    spacerColorWidget.withProperties(
        Map.of("colorWhenTrue", getColorMatchString(colorSensorReader.getSpacerValues().color)));
    fullColorWidget.withProperties(
        Map.of("colorWhenTrue", getColorMatchString(colorSensorReader.getFullValues().color)));
  }

  private String getColorMatchString(Color color) {
    var colorMatchResult = colorMatch.matchClosestColor(color);
    String intakeColorString = "Black";
    if (colorMatchResult.color == COLOR_NONE){
      intakeColorString = "Black";
    } else if(colorMatchResult.color == COLOR_RED){
      intakeColorString = "Red";
    } else if(colorMatchResult.color == COLOR_BLUE){
      intakeColorString = "Blue";
    }
    return intakeColorString;
  }

  /**
   * Returns the color detected by the intake sensor
   * @return the color detected
   */
  public Color getIntakeColor() {
    return colorMatch.matchClosestColor(colorSensorReader.getIntakeValues().color).color;
  }

  /**
   * Returns the color detected by the spacer sensor
   * @return the color detected
   */
  public Color getSpacerColor() {
     return colorMatch.matchClosestColor(colorSensorReader.getSpacerValues().color).color;
  }

  /**
   * Returns the color detected by the full sensor
   * @return the color detected
   */
  public Color getFullColor() {
    return colorMatch.matchClosestColor(colorSensorReader.getFullValues().color).color;
  }

  /**
   * Returns true when the full sensor is tripped. The falling edge is debounced.
   * @return true if the full sensor is tripped, otherwise false
   */
  public boolean isFullSensorTripped() {
    var sensorTripped = colorMatch.matchClosestColor(colorSensorReader.getFullValues().color).color != COLOR_NONE;
    return fullSensorDebouncer.calculate(sensorTripped);
  }

  private boolean isIntakeSensorTripped() {
    return colorSensorReader.getIntakeValues().proximity > THRESHOLD_INTAKE;
  }

  public boolean isSpacerSensorTripped() {
    return colorSensorReader.getSpacerValues().proximity > THRESHOLD_SPACE;
  }

  public int getCargoCount() {
    return cargoCount;
  }

  public void resetCargoCount(int cargoCount) {
    this.cargoCount = cargoCount;
  }

  public void load() {
    pidController.setReference(IndexerConstants.BELT_RUN_SPEED, ControlType.kVelocity);
  }

  public void unload() {
    indexer.set(-.5);
  }

  public void shoot() {
    pidController.setReference(IndexerConstants.BELT_SHOOT_SPEED, ControlType.kVelocity);
  }

  public void stop() {
    pidController.setReference(0, ControlType.kVelocity);
  }

}