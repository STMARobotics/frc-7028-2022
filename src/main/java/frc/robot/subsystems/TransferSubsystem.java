package frc.robot.subsystems;

import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER_FOLLOWER;
import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER_LEADER;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {

  private final WPI_TalonSRX transferLeader = new WPI_TalonSRX(DEVICE_ID_TRANSFER_LEADER);
  private final WPI_TalonSRX transferFollower = new WPI_TalonSRX(DEVICE_ID_TRANSFER_FOLLOWER);

  public TransferSubsystem() {
    transferFollower.setInverted(true);
    transferFollower.follow(transferLeader);
  }

  public void set(double power) {
    transferLeader.set(power);
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

  @Override
  public void periodic() {

  }

}
