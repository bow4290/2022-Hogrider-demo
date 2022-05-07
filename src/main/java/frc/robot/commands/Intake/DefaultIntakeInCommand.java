package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeInCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private DoubleSupplier intakeActiveDouble;

  public DefaultIntakeInCommand(DoubleSupplier intakeActiveDouble,IntakeSubsystem intakeSubsystem) {
    this.intakeActiveDouble = intakeActiveDouble;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (intakeActiveDouble.getAsDouble() > IntakeConstants.intakeTriggerbuffer){
      intakeSubsystem.intakeSpin(1);
    } else {
      intakeSubsystem.intakeSpin(0);
    }

  }

  @Override
  public void end(boolean interrupted) { 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}