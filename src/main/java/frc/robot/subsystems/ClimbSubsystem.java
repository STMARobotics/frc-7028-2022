package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_FIRST_STAGE_CLIMB;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX firstStageClimb = new WPI_TalonFX(DEVICE_ID_FIRST_STAGE_CLIMB);

  public ClimbSubsystem() {
    firstStageClimb.configFactoryDefault();
    firstStageClimb.setSafetyEnabled(true);
    firstStageClimb.setNeutralMode(NeutralMode.Brake);
  }

  public void setFirstStage(double power) {
    firstStageClimb.set(power); 
  }

  public void stopFirstStage() {
    setFirstStage(0);
  }
}
