package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX belt = new WPI_TalonSRX(DEVICE_ID_BELT);

  public IndexerSubsystem() {
    belt.configFactoryDefault();
    belt.setNeutralMode(NeutralMode.Coast);
  }

  public void set(double power) {
    belt.set(power);
  }

  public void intake() {
    set(.50);
  }

  public void output() {
    set(-.50);
  }

  public void stop() {
    set(0);
  }

}