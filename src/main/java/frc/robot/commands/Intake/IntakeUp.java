package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUp extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public IntakeUp(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() { intakeSubsystem.retractIntake(); }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return true; }
}
