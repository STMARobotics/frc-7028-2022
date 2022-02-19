package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * RumbleCommand
 */
public class RumbleCommand extends CommandBase {

  private final XboxController controller;
  private final RumbleType rumbleType;

  public RumbleCommand(XboxController controller, RumbleType rumbleType) {
    this.controller = controller;
    this.rumbleType = rumbleType;
  }

  @Override
  public void initialize() {
    controller.setRumble(rumbleType, 1);
  }

  @Override
  public void end(boolean interrupted) {
    controller.setRumble(rumbleType, 0);
  }

}