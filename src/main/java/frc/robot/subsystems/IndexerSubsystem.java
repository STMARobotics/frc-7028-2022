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
    // A small amount of ramp helps prevent voltage drop when the indexer starts up. The voltage drop was causing the
    // shooter flywheel to slow down.
    indexer.configOpenloopRamp(0.1);
  }

  public void load() {
    indexer.set(.3);
  }

  public void unload() {
    indexer.set(-.3);
  }

  public void shoot() {
    indexer.set(.4);
  }

  public void stop() {
    indexer.set(0);
  }

}