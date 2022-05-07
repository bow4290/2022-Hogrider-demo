package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOut extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public IntakeOut(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.intakeSpin(-1);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intakeSpin(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}