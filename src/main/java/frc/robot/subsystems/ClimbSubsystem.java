package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_FIRST_STAGE_CLIMB;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX firstStageClimb = new WPI_TalonFX(DEVICE_ID_FIRST_STAGE_CLIMB);

  // Indicator that soft limits have been temporarily disabled
  private boolean limitsDisabled = false;

  public ClimbSubsystem() {
    firstStageClimb.configFactoryDefault();
    firstStageClimb.setSafetyEnabled(true);
    firstStageClimb.setNeutralMode(NeutralMode.Brake);
    firstStageClimb.configForwardSoftLimitEnable(true);
    firstStageClimb.configReverseSoftLimitEnable(true);
    firstStageClimb.configForwardSoftLimitThreshold(ClimbConstants.SOFT_LIMIT_FIRST_STAGE_FWD);
    firstStageClimb.configReverseSoftLimitThreshold(0);
  }

  public void setFirstStage(double power) {
    if (limitsDisabled) {
      // Reduce speed when limits disabled
      firstStageClimb.set(power * .25);
    } else {
      firstStageClimb.set(power); 
    }
  }

  public void stopFirstStage() {
    setFirstStage(0);
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
    firstStageClimb.setSelectedSensorPosition(ClimbConstants.SOFT_LIMIT_FIRST_STAGE_REV);
    firstStageClimb.configReverseSoftLimitEnable(true);
    firstStageClimb.configForwardSoftLimitEnable(true);
  }
}
