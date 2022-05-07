package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class DisableTurretCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;

  public DisableTurretCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;

    // addRequirements(turretSubsystem);  
  }

  @Override
  public void initialize() {
    turretSubsystem.isTurretStopped = true;
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
