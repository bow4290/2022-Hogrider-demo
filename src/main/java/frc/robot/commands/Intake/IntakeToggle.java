package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStatus;

public class IntakeToggle extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public IntakeToggle(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    if (IntakeSubsystem.getIntakePosition() == IntakeStatus.UP) {
      intakeSubsystem.extendIntake();
    } else {
      intakeSubsystem.retractIntake();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return true; }
}
