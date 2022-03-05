package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.DEVICE_ID_INTAKE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor = new CANSparkMax(DEVICE_ID_INTAKE, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
  }
  
  public void intake() {
    intakeMotor.set(.8);
  }
  
  public void reverse() {
    intakeMotor.set(-.5);
  }
  
  public void stop() {
    intakeMotor.set(0);
  }
  
}
