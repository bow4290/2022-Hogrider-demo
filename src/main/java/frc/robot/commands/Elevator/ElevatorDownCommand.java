package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevatorSubsystem.getEncoderposition();

    if (elevatorSubsystem.getEncoderposition() <= ElevatorConstants.reverseSoftLimit) {
      elevatorSubsystem.stopElevator();
    } else {    
      elevatorSubsystem.retractElevator(ElevatorConstants.elevatorSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopElevator();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
