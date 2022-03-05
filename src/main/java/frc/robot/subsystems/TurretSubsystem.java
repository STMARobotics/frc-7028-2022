package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the turret.
 */
public class TurretSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(DEVICE_ID_TURRET);

  public TurretSubsystem() {
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configVoltageCompSaturation(12);
    turretMotor.enableVoltageCompensation(true);
    turretMotor.setSafetyEnabled(true);
  }

  public void run(double speed) {
    turretMotor.set(speed);
  }

  public void stop() {
    turretMotor.set(0);
  }

}
