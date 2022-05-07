package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveForDistanceCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  double inches;

  public AutoDriveForDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double inches) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.inches = inches;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetDriveEncoders();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(DriveConstants.autonomousDriveSpeed, DriveConstants.autonomousDriveSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(0,0);
    drivetrainSubsystem.resetDriveEncoders();
  }

  @Override
  public boolean isFinished() {
    return (drivetrainSubsystem.getRightCalculatedPosition() >= inches);
  }
}
