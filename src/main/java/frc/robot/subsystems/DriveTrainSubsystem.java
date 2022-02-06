package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DrivetrainConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DrivetrainConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DrivetrainConstants.RIGHT_LEADER_ID;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftLeader = new WPI_TalonFX(LEFT_LEADER_ID);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(LEFT_FOLLOWER_ID);
  private final WPI_TalonFX rightLeader = new WPI_TalonFX(RIGHT_LEADER_ID);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(RIGHT_FOLLOWER_ID);
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
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

}
