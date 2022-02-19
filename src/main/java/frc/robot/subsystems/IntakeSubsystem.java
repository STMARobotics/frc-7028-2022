package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX intake = new WPI_TalonSRX(7);

  public IntakeSubsystem() {
    
  }

  public void set(double power) {
    intake.set(power);
  }

  public void stop() {
    set(0);
  }

  public void fullBack() {
    set(-1);
  }

  public void halfBack() {
    set(-.5);
  }

  public void fullForward() {
    set(1);
  }

  public void halfForward() {
    set(.5);
  }

  @Override
  public void periodic() {

  }
  
}
  
