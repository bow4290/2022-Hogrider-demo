package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private DoubleSupplier leftSpeed;
  private DoubleSupplier rightSpeed;

  public DefaultDriveCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DrivetrainSubsystem drivetrainSubsystem) {
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}