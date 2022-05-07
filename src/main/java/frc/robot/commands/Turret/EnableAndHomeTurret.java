package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class EnableAndHomeTurret extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final double position;

  public EnableAndHomeTurret(TurretSubsystem turretSubsystem, double position) {
    this.turretSubsystem = turretSubsystem;
    this.position = position;
  }

  @Override
  public void initialize() {
    turretSubsystem.isTurretStopped = false;
    turretSubsystem.homingPosition = position;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
