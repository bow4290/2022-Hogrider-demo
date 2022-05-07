package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootManual extends CommandBase {
  private ShooterSubsystem shooterSubsystem;

  public ShootManual(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shooterSubsystem.manualShoot();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shoot(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
