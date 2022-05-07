package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurnRightAngleCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  double degrees;

  public AutoTurnRightAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double degrees) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.degrees = degrees;
    
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetDriveEncoders();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(DriveConstants.autonomousTurnSpeed, -DriveConstants.autonomousTurnSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(0,0);
  }

  @Override
  public boolean isFinished() {
    return (drivetrainSubsystem.getTurnRightDegrees() >= degrees);
  }
}
