package frc.robot.subsystems;

import static frc.robot.Constants.ArcadeConstants.MAX_ANGULAR_VEL_ARCADE;
import static frc.robot.Constants.ArcadeConstants.MAX_SPEED_ARCADE;
import static frc.robot.Constants.ArcadeConstants.ROTATE_RATE_LIMIT_ARCADE;
import static frc.robot.Constants.ArcadeConstants.SPEED_RATE_LIMIT_ARCADE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_FOLLOWER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_LEADER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_FOLLOWER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_LEADER;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.EDGES_PER_ROTATION;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;

/**
 * DriveTrainSubsystem
 */
public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftLeader = new WPI_TalonFX(DEVICE_ID_LEFT_LEADER);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(DEVICE_ID_LEFT_FOLLOWER);
  private final WPI_TalonFX rightLeader = new WPI_TalonFX(DEVICE_ID_RIGHT_LEADER);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(DEVICE_ID_RIGHT_FOLLOWER);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;

  private SlewRateLimiter speedRateLimiter = new SlewRateLimiter(SPEED_RATE_LIMIT_ARCADE);
  private SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATE_RATE_LIMIT_ARCADE);
  
  public DriveTrainSubsystem() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.slot0.kP = DriveTrainConstants.kP;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = DriveTrainConstants.OPEN_LOOP_RAMP;

    rightLeader.configAllSettings(talonConfig);
    rightLeader.enableVoltageCompensation(true);
    rightLeader.enableVoltageCompensation(true);
    rightFollower.configFactoryDefault();
    leftLeader.configAllSettings(talonConfig);
    leftLeader.enableVoltageCompensation(true);
    leftFollower.configFactoryDefault();

    rightLeader.setSafetyEnabled(true);
    leftLeader.setSafetyEnabled(true);
    
    setNeutralMode(NeutralMode.Coast);
    
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    rightLeader.overrideLimitSwitchesEnable(false);
    leftLeader.overrideLimitSwitchesEnable(false);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    
    new Trigger(RobotState::isEnabled).whenActive(new StartEndCommand(() -> {
      leftLeader.setNeutralMode(NeutralMode.Brake);
      leftFollower.setNeutralMode(NeutralMode.Brake);
      rightLeader.setNeutralMode(NeutralMode.Brake);
      rightFollower.setNeutralMode(NeutralMode.Brake);
    }, () -> {
      leftLeader.setNeutralMode(NeutralMode.Coast);
      leftFollower.setNeutralMode(NeutralMode.Coast);
      rightLeader.setNeutralMode(NeutralMode.Coast);
      rightFollower.setNeutralMode(NeutralMode.Coast);
    }));
    setCurrentPose(new Pose2d());
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    dashboard.addString("Pose", () -> differentialDriveOdometry.getPoseMeters().toString());
    dashboard.addNumber("Speed", () ->
        DRIVE_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
          edgesPerDecisecToMetersPerSec(leftLeader.getSelectedSensorVelocity()),
          edgesPerDecisecToMetersPerSec(rightLeader.getSelectedSensorVelocity()))).vxMetersPerSecond);
    dashboard.addNumber("Rotation", () ->
        DRIVE_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
          edgesPerDecisecToMetersPerSec(leftLeader.getSelectedSensorVelocity()),
          edgesPerDecisecToMetersPerSec(rightLeader.getSelectedSensorVelocity()))).omegaRadiansPerSecond);
  }

  public void addInstruments(Orchestra orchestra) {
    orchestra.addInstrument(rightLeader);
    orchestra.addInstrument(rightFollower);
    orchestra.addInstrument(leftLeader);
    orchestra.addInstrument(leftFollower);
  }

  /**
   * Resets the current pose to 0, 0, 0° and resets the saved pose
   */
  public void resetOdometry() {
    setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the current pose to the specified pose. This this ONLY be called
   * when the robot's position on the field is known, like at the beginnig of
   * a match. This will also reset the saved pose since the old pose could be invalidated.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    zeroDriveTrainEncoders();
    savedPose = newPose;
    differentialDriveOdometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        edgesToMeters(getLeftEncoderPosition()),
        edgesToMeters(getRightEncoderPosition()));
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed      speed along the x axis [-1.0..1.0]
   * @param rotation   rotation rate along the z axis [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void arcadeDrive(double speed, double rotation, boolean useSquares) {
    var xSpeed = speed;
    var zRotation = rotation;
    if (useSquares) {
      xSpeed *= Math.abs(xSpeed);
      zRotation *= Math.abs(zRotation);
    }
    xSpeed = speedRateLimiter.calculate(safeClamp(speed));
    zRotation = -rotationRateLimiter.calculate(safeClamp(rotation));
    xSpeed *= MAX_SPEED_ARCADE;
    zRotation *= MAX_ANGULAR_VEL_ARCADE;
    var wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRotation));
    tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean useSquares) {
    var xLeftSpeed = safeClamp(leftSpeed) * MAX_SPEED_ARCADE;
    var xRightSpeed = safeClamp(rightSpeed) * MAX_SPEED_ARCADE;
    if (useSquares) {
      xLeftSpeed *= Math.abs(xLeftSpeed);
      xRightSpeed *= Math.abs(xRightSpeed);
    }
    tankDriveVelocity(xLeftSpeed, xRightSpeed);
  }

  /**
   * Controls the left and right side of the drive using Talon SRX closed-loop
   * velocity.
   * 
   * @param leftVelocity  left velocity in meters per second
   * @param rightVelocity right velocity in meters per second
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftFeedForwardVolts = FEED_FORWARD.calculate(leftVelocity);
    var rightFeedForwardVolts = FEED_FORWARD.calculate(rightVelocity);

    leftLeader.set(
        ControlMode.Velocity, 
        metersPerSecToEdgesPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    rightLeader.set(
        ControlMode.Velocity,
        metersPerSecToEdgesPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
  }

  /**
   * Sets the drivetrain to zero velocity and rotation.
   */
  public void stop() {
    leftLeader.set(0);
    rightLeader.set(0);
    speedRateLimiter.reset(0);
    rotationRateLimiter.reset(0);
  }

  /**
   * Returns value clamped between [-1, 1]. Not-a-number (NaN) returns 0;
   * @param input value to clamp
   * @return clamped value
   */
  private double safeClamp(double input) {
    if (Double.isNaN(input)) {
      return 0;
    }
    return MathUtil.clamp(input, -1, 1);
  }

  /**
   * Sets the neutral mode for the drive train
   * 
   * @param neutralMode the desired neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    leftLeader.setNeutralMode(neutralMode);
    leftFollower.setNeutralMode(neutralMode);
    rightLeader.setNeutralMode(neutralMode);
    rightFollower.setNeutralMode(neutralMode);
  }

  /**
   * returns left encoder position
   * 
   * @return left encoder position
   */
  public double getLeftEncoderPosition() {
    return leftLeader.getSelectedSensorPosition();
  }

  /**
   * returns right encoder position
   * 
   * @return right encoder position
   */
  public double getRightEncoderPosition() {
    return rightLeader.getSelectedSensorPosition();
  }

  private void zeroDriveTrainEncoders() {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  public Pose2d getCurrentPose() {
    return differentialDriveOdometry.getPoseMeters();
  }

  public void saveCurrentPose() {
    savedPose = getCurrentPose();
  }

  public Pose2d getSavedPose() {
    return savedPose;
  }

  /**
   * Returns the heading of the robot in form required for odometry.
   *
   * @return the robot's heading in degrees, from -180 to 180 with positive value
   *         for left turn.
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory) {
    return new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA),
            DriveTrainConstants.DRIVE_KINEMATICS,
            this::tankDriveVelocity,
            this);
  }

  /**
   * Converts from encoder edges to meters.
   * 
   * @param edges encoder edges to convert
   * @return meters
   */
  public static double edgesToMeters(double edges) {
    return (WHEEL_CIRCUMFERENCE_METERS / (EDGES_PER_ROTATION * DRIVE_GEAR_RATIO)) * edges;
  }

  /**
   * Converts from encoder edges per 100 milliseconds to meters per second.
   * @param stepsPerDecisec edges per decisecond
   * @return meters per second
   */
  public static double edgesPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return edgesToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts from meters to encoder edges.
   * @param meters meters
   * @return encoder edges
   */
  public static double metersToEdges(double meters) {
    return (meters / WHEEL_CIRCUMFERENCE_METERS) * (EDGES_PER_ROTATION * DRIVE_GEAR_RATIO);
  }

  /**
   * Converts from meters per second to encoder edges per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder edges per decisecond
   */
  public static double metersPerSecToEdgesPerDecisec(double metersPerSec) {
    return metersToEdges(metersPerSec) * .1d;
  }
}