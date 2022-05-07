package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDown extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public IntakeDown(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.extendIntake();
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