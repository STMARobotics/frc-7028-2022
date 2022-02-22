package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX indexer = new WPI_TalonSRX(DEVICE_ID_INDEXER);

  public IndexerSubsystem() {
    indexer.configFactoryDefault();
    indexer.setNeutralMode(NeutralMode.Coast);
    indexer.setInverted(true);
  }

  public void set(double power) {
    indexer.set(power);
  }

  public void load() {
    set(.65);
  }

  public void unload() {
    set(-.65);
  }

  public void stop() {
    set(0);
  }

}