package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setIdleMode(IdleMode.kCoast);
    // A small amount of ramp helps prevent voltage drop when the indexer starts up. The voltage drop was causing the
    // shooter flywheel to slow down.
    indexer.setOpenLoopRampRate(0.1);
    indexer.burnFlash();
  }

  public void load() {
    indexer.set(.5);
  }

  public void unload() {
    indexer.set(-.5);
  }

  public void shoot() {
    indexer.set(.7);
  }

  public void stop() {
    indexer.set(0);
  }

}