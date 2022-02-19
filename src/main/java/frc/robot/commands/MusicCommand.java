package frc.robot.commands;

import java.io.File;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MusicCommand extends CommandBase {
  
  private Orchestra orchestra;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public MusicCommand(DriveTrainSubsystem driveTrainSubsystem, ShooterSubsystem shooterSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(driveTrainSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    orchestra = new Orchestra();
    orchestra.loadMusic(new File(Filesystem.getDeployDirectory(), "unity.chrp").getAbsolutePath());
    driveTrainSubsystem.addInstruments(orchestra);
    shooterSubsystem.addInstruments(orchestra);
    orchestra.play();
  }

  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

}
