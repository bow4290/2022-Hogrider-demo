package frc.robot.commands.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodExtendCommand extends CommandBase {
  HoodSubsystem hoodSubsystem;

  public HoodExtendCommand(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;

    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    hoodSubsystem.extendHoodSolenoid();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
