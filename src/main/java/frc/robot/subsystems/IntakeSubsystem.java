package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.CHANNEL_SOLENOID_BACKWARD;
import static frc.robot.Constants.IntakeConstants.CHANNEL_SOLENOID_FORWARD;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_INTAKE;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor = new CANSparkMax(DEVICE_ID_INTAKE, MotorType.kBrushless);
  private final Compressor compressor = new Compressor(REVPH);
  private final DoubleSolenoid intakeSolenoid =
      new DoubleSolenoid(REVPH, CHANNEL_SOLENOID_FORWARD, CHANNEL_SOLENOID_BACKWARD);

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setOpenLoopRampRate(.1);
    intakeMotor.burnFlash();

    new Trigger(RobotState::isEnabled).onTrue(new InstantCommand(compressor::enableDigital));
  }

  public void intake() {
    intakeMotor.set(.5d);
  }
  
  public void reverse() {
    intakeMotor.set(-.5d);
  }
  
  public void stop() {
    intakeMotor.set(0);
  }

  public void deploy() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retract() {
    intakeSolenoid.set(Value.kReverse);
  }
  
}
