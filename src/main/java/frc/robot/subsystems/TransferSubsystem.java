package frc.robot.subsystems;

import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {

  private final CANSparkMax transferMotor = new CANSparkMax(DEVICE_ID_TRANSFER, MotorType.kBrushless);

  public TransferSubsystem() {
    transferMotor.restoreFactoryDefaults();
    transferMotor.enableVoltageCompensation(12);
    transferMotor.setIdleMode(IdleMode.kCoast);
    transferMotor.setInverted(true);
    transferMotor.setOpenLoopRampRate(.1);
    transferMotor.burnFlash();
  }

  public void set(double power) {
    transferMotor.set(power);
  }

  public void intake() {
    set(.8);
  }

  public void output() {
    set(-.8);
  }

  public void stop() {
    set(0);
  }

}
