package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class ManualTurretCounterClockwiseCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;

  public ManualTurretCounterClockwiseCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    turretSubsystem.turnTurret(-TurretConstants.manualTurnSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.turnTurret(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
