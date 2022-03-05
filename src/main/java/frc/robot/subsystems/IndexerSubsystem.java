package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, MotorType.kBrushless);
  private final RelativeEncoder indexerEncoder;
  private final SparkMaxPIDController pidController;

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setIdleMode(IdleMode.kCoast);
    indexer.setOpenLoopRampRate(0.1);
    indexerEncoder = indexer.getAlternateEncoder(Type.kQuadrature, 8192);
    pidController = indexer.getPIDController();
    pidController.setFeedbackDevice(indexerEncoder);
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