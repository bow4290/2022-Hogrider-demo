package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public IntakeStop(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() { intakeSubsystem.intakeSpin(0); }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return true; }
}