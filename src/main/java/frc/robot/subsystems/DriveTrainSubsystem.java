package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.DEVICE_ID_LEFT_FOLLOWER;
import static frc.robot.Constants.DrivetrainConstants.DEVICE_ID_LEFT_LEADER;
import static frc.robot.Constants.DrivetrainConstants.DEVICE_ID_RIGHT_FOLLOWER;
import static frc.robot.Constants.DrivetrainConstants.DEVICE_ID_RIGHT_LEADER;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftLeader = new WPI_TalonFX(DEVICE_ID_LEFT_LEADER);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(DEVICE_ID_LEFT_FOLLOWER);
  private final WPI_TalonFX rightLeader = new WPI_TalonFX(DEVICE_ID_RIGHT_LEADER);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(DEVICE_ID_RIGHT_FOLLOWER);
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);
  

  /** Creates a new ExampleSubsystem. */
  public DriveTrainSubsystem() {
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();

    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);

    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    rightLeader.setInverted(true);
    rightFollower.setInverted(true);

    rightLeader.configOpenloopRamp(0.2);
    leftLeader.configOpenloopRamp(0.2);
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
    drive.arcadeDrive(speed, rotation, squareInputs);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void addInstruments(Orchestra orchestra) {
    orchestra.addInstrument(rightLeader);
    orchestra.addInstrument(rightFollower);
    orchestra.addInstrument(leftLeader);
    orchestra.addInstrument(leftFollower);
  }

}
