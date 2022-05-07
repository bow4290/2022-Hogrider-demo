package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.TravArmStatus;

public class ArmToggle extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public ArmToggle(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    if (ElevatorSubsystem.getTravArmStatus() == TravArmStatus.UP) {
      elevatorSubsystem.extendArm();
    } else {
      elevatorSubsystem.retractArm();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
