package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class EnableTurretCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;

  public EnableTurretCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;

    // addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.isTurretStopped = false;
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
