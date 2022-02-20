package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.DEVICE_ID_INTAKE;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX intake = new WPI_TalonSRX(DEVICE_ID_INTAKE);

  public IntakeSubsystem() {
    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Coast);
  }

  public void set(double power) {
    intake.set(power);
  }
  
  public void forward() {
    set(.5);
  }
  
  public void reverse() {
    set(-.5);
  }
  
  public void stop() {
    set(0);
  }
  
}