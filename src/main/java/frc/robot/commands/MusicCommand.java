package frc.robot.commands;

import java.io.File;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class MusicCommand extends CommandBase {
  
  private Orchestra orchestra;
  private final DriveTrainSubsystem driveTrainSubsystem;

  public MusicCommand(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    orchestra = new Orchestra();
    orchestra.loadMusic(new File(Filesystem.getDeployDirectory(), "unity.chrp").getAbsolutePath());
    driveTrainSubsystem.addInstruments(orchestra);
    orchestra.play();
  }

  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

}
