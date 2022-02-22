package frc.robot.subsystems;

/**
 * Limelight profiles enum
 */
public enum Profile {

  NEAR(0.0),
  FAR(1.0);

  /** ID for the Limelight profile */
  public final double pipelineId;

  /**
   * Create a limelight profile
   * @param pipelineId pipeline ID
   */
  private Profile(double pipelineId) {
    this.pipelineId = pipelineId;
  }

}