package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_FIRST_STAGE_CLIMB;
import static frc.robot.Constants.ClimbConstants.SOFT_LIMIT_FIRST_STAGE_FWD;
import static frc.robot.Constants.ClimbConstants.SOFT_LIMIT_FIRST_STAGE_REV;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX firstStageClimb = new WPI_TalonFX(DEVICE_ID_FIRST_STAGE_CLIMB);
  private final BooleanSupplier isTurretClear;

  // Indicator that soft limits have been temporarily disabled
  private boolean limitsDisabled = false;


  public ClimbSubsystem(BooleanSupplier isTurretClear) {
    this.isTurretClear = isTurretClear;

    firstStageClimb.configFactoryDefault();
    firstStageClimb.setSafetyEnabled(true);
    firstStageClimb.setNeutralMode(NeutralMode.Brake);
    firstStageClimb.configForwardSoftLimitEnable(true);
    firstStageClimb.configReverseSoftLimitEnable(true);
    firstStageClimb.configForwardSoftLimitThreshold(SOFT_LIMIT_FIRST_STAGE_FWD);
    firstStageClimb.configReverseSoftLimitThreshold(SOFT_LIMIT_FIRST_STAGE_REV);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailLayout = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 2)).withPosition(0, 0);
    detailLayout.addBoolean("Turret Clear", isTurretClear).withPosition(0, 0);
    detailLayout.addBoolean("Fwd Limit", () -> firstStageClimb.getSelectedSensorPosition() <= SOFT_LIMIT_FIRST_STAGE_FWD)
        .withPosition(1, 0);
    detailLayout.addNumber("Native Position", firstStageClimb::getSelectedSensorPosition).withPosition(0, 1);
    detailLayout.addBoolean("Rev Limit", () -> firstStageClimb.getSelectedSensorPosition() >= 0).withPosition(1, 1);
    dashboard.add(this).withPosition(0, 1);
  }

  public void addDriverDashboardWidgets(ShuffleboardTab driverTab) {
    driverTab.addBoolean("Climb Down", () -> !this.isFirstStageRaised()).withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(11, 0).withSize(0, 0);
  }

  public void setFirstStage(double power) {
    if (isTurretClear.getAsBoolean()) {
      if (limitsDisabled) {
        // Reduce speed when limits disabled
        firstStageClimb.set(power * .25);
      } else {
        firstStageClimb.set(power); 
      }
    } else {
      firstStageClimb.stopMotor();
    }
  }

  public void stopFirstStage() {
    firstStageClimb.stopMotor();
  }

  public boolean isFirstStageRaised() {
    return firstStageClimb.getSelectedSensorPosition() > 3000;
  }

  /**
   * Temporarily disable limits to allow the climb to be calibrated. Use
   * {@link #resetAndEnableLimits()} when the climb is at its new reverse limit
   */
  public void disableLimits() {
    limitsDisabled = true;
    firstStageClimb.configReverseSoftLimitEnable(false);
    firstStageClimb.configForwardSoftLimitEnable(false);
  }

  /**
   * Resets the reverse limit to the current position and enables the limits
   */
  public void resetAndEnableLimits() {
    limitsDisabled = false;
    firstStageClimb.setSelectedSensorPosition(SOFT_LIMIT_FIRST_STAGE_REV);
    firstStageClimb.configReverseSoftLimitEnable(true);
    firstStageClimb.configForwardSoftLimitEnable(true);
  }
}
