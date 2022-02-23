package frc.robot.subsystems;

import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER_FOLLOWER;
import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER_LEADER;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {

  private final WPI_TalonSRX transferLeader = new WPI_TalonSRX(DEVICE_ID_TRANSFER_LEADER);
  private final WPI_TalonSRX transferFollower = new WPI_TalonSRX(DEVICE_ID_TRANSFER_FOLLOWER);

  public TransferSubsystem() {
    transferLeader.configFactoryDefault();
    transferFollower.configFactoryDefault();
    transferLeader.setNeutralMode(NeutralMode.Coast);
    transferFollower.setNeutralMode(NeutralMode.Coast);

    transferFollower.follow(transferLeader);
    transferFollower.setInverted(true);
    transferLeader.setInverted(true);
  }

  public void set(double power) {
    transferLeader.set(power);
  }

  public void intake() {
    set(.25);
  }

  public void output() {
    set(-.25);
  }

  public void stop() {
    set(0);
  }

}
