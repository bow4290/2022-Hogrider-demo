package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.BallIdentification;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeIn extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private BallIdentification ballUpper;
  private BallIdentification ballLower;

  public AutoIntakeIn(IntakeSubsystem intakeSubsystem, BallIdentification ballUpper, BallIdentification ballLower) {
    this.intakeSubsystem = intakeSubsystem;
    this.ballUpper = ballUpper;
    this.ballLower = ballLower;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.intakeSpin(1);
  }

  @Override
  public void end(boolean interrupted) { 
    intakeSubsystem.intakeSpin(0);
  }

  @Override
  public boolean isFinished() {
    return (ballLower.isBallPresent() && ballUpper.isBallPresent());
  }
}