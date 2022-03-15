package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_FIRST_STAGE_CLIMB;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_SECOND_STAGE_CLIMB;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX firstStageClimb = new WPI_TalonFX(DEVICE_ID_FIRST_STAGE_CLIMB);
  private final WPI_TalonFX secondStageClimb = new WPI_TalonFX(DEVICE_ID_SECOND_STAGE_CLIMB);

  public ClimbSubsystem() {
    
  }

  public void setFirstStage(double power) {
    firstStageClimb.set(power); 
  }

  public void setSecondStage(double power) {
    secondStageClimb.set(power);
  }

  public void stopFirstStage() {
    setFirstStage(0);
  }

  public void stopSecondStage() {
    setSecondStage(0);
  }

}
