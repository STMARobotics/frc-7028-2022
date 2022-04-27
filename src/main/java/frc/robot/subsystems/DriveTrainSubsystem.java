package frc.robot.subsystems;

import static edu.wpi.first.hal.simulation.SimDeviceDataJNI.getSimDeviceHandle;
import static edu.wpi.first.hal.simulation.SimDeviceDataJNI.getSimValueHandle;
import static frc.robot.Constants.ArcadeConstants.MAX_ANGULAR_VEL_ARCADE;
import static frc.robot.Constants.ArcadeConstants.MAX_SPEED_ARCADE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_FOLLOWER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_LEADER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_FOLLOWER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_LEADER;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.EDGES_PER_ROTATION;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.DriveTrainConstants.TRACK_WIDTH_METERS;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_DIAMETER_INCHES;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Field2d field2d = new Field2d();

  // Simulation objects
  private final TalonFXSimCollection rightSimCollection = rightLeader.getSimCollection();
  private final TalonFXSimCollection leftSimCollection = leftLeader.getSimCollection();
  private final LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
    DriveTrainConstants.kV, DriveTrainConstants.kA, DriveTrainConstants.kV_ANGULAR, DriveTrainConstants.kA_ANGULAR);
  private final DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(
      drivetrainSystem, DCMotor.getFalcon500(2), DRIVE_GEAR_RATIO, TRACK_WIDTH_METERS, WHEEL_DIAMETER_INCHES / 2, null);
  SimDouble simGyroAngle = new SimDouble(getSimValueHandle(getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
  
  public DriveTrainSubsystem() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    resetOdometry();

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.slot0.kP = DriveTrainConstants.kP;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = DriveTrainConstants.OPEN_LOOP_RAMP;

    rightLeader.configAllSettings(talonConfig);
    rightFollower.configFactoryDefault();
    leftLeader.configAllSettings(talonConfig);

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
    SmartDashboard.putData(field2d);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailLayout = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 2, "Number of rows", 2)).withPosition(0, 0);
    detailLayout.addNumber("Speed", () -> getCurrentChassisSpeeds().vxMetersPerSecond).withPosition(0, 0);
    detailLayout.addNumber("Rotation", () -> getCurrentChassisSpeeds().omegaRadiansPerSecond).withPosition(1, 0);
    detailLayout.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 1);
    detailLayout.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 1);
    dashboard.add(this).withPosition(0, 1);
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", pose.getX(), pose.getY());
  }

  /**
   * Resets the current pose to 0, 0, 0°
   */
  public void resetOdometry() {
    setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginnig of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    zeroDriveTrainEncoders();
    differentialDriveOdometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        edgesToMeters(getLeftEncoderPosition()),
        edgesToMeters(getRightEncoderPosition()));
    
    field2d.setRobotPose(getCurrentPose());
  }

  @Override
  public void simulationPeriodic() {
    // When in simulation, update the drivetrain simulation object, then use it to update the encoder speed and
    // position, and the gyro angle
    leftSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
    rightSimCollection.setBusVoltage(RobotController.getBatteryVoltage());

    // Update the drive simulator, then it will give us encoder and gyro values
    drivetrainSimulator.setInputs(
        leftSimCollection.getMotorOutputLeadVoltage(),
        -rightSimCollection.getMotorOutputLeadVoltage());
    drivetrainSimulator.update(0.02);

    // Set encoder speed and position
    leftSimCollection.setIntegratedSensorRawPosition((int) metersToEdges(drivetrainSimulator.getLeftPositionMeters()));
    leftSimCollection.setIntegratedSensorVelocity(
        (int) metersPerSecToEdgesPerDecisec(drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    rightSimCollection.setIntegratedSensorRawPosition(
        -(int) metersToEdges(drivetrainSimulator.getRightPositionMeters()));
    rightSimCollection.setIntegratedSensorVelocity(
        -(int) metersPerSecToEdgesPerDecisec(drivetrainSimulator.getRightVelocityMetersPerSecond()));

    // Set gyro Yaw
    // NavX expects clockwise positive, but sim outputs clockwise negative
    simGyroAngle.set(Math.IEEEremainder(-drivetrainSimulator.getHeading().getDegrees(), 360));
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed      speed along the x axis [-1.0..1.0] (FWD is +)
   * @param rotation   rotation rate along the z axis [-1.0..1.0] (CW is +)
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void arcadeDrive(double speed, double rotation, boolean useSquares) {
    var xSpeed = safeClamp(speed);
    var zRotation = safeClamp(rotation);
    if (useSquares) {
      xSpeed *= Math.abs(xSpeed);
      zRotation *= Math.abs(zRotation);
    }
    xSpeed *= MAX_SPEED_ARCADE;
    zRotation *= MAX_ANGULAR_VEL_ARCADE;
    // Note that CCW is positive rotation for the ChasisSpeeds object, so we negate it
    var wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -zRotation));
    tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Gets the current ChassisSpeeds in meters per second.
   * Use {@link #getVelocityPercent()} or {@link #getAngularVelocityPercent()} to get speeds in percentage terms.
   * @return chasis speed of the drivetrain
   */
  public ChassisSpeeds getCurrentChassisSpeeds() {
    return DRIVE_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
        edgesPerDecisecToMetersPerSec(leftLeader.getSelectedSensorVelocity()),
        edgesPerDecisecToMetersPerSec(rightLeader.getSelectedSensorVelocity())));
  }

  /**
   * Gets the current velocity as a percentage of the max speed.
   * Use {@link #getCurrentChassisSpeeds()} to get speeds in meters/second.
   * @return current velocity percent
   */
  public double getVelocityPercent() {
    return getCurrentChassisSpeeds().vyMetersPerSecond / MAX_SPEED_ARCADE;
  }

  /**
   * Gets the current angular velocity (rotation) as a percentage of the max angular velocity.
   * Use {@link #getCurrentChassisSpeeds()} to get speeds in meters/second.
   * @return current angular velocity percent
   */
  public double getAngularVelocityPercent() {
    return getCurrentChassisSpeeds().omegaRadiansPerSecond / MAX_ANGULAR_VEL_ARCADE;
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
        leftFeedForwardVolts / RobotController.getBatteryVoltage());
    rightLeader.set(
        ControlMode.Velocity,
        metersPerSecToEdgesPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / RobotController.getBatteryVoltage());
  }

  /**
   * Sets the drivetrain to zero velocity and rotation.
   */
  public void stop() {
    leftLeader.set(0);
    rightLeader.set(0);
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
    return edgesToMeters(stepsPerDecisec / .1d);
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