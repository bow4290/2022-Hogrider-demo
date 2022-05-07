package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    // UsbCamera camera0 = CameraServer.startAutomaticCapture();
    // camera0.setVideoMode(PixelFormat.kMJPEG, 256, 192, 30);
  }

  @Override
  public void robotPeriodic() {
    robotContainer.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    
    // if (autonomousCommand != null) {
      robotContainer.turretSubsystem.isTurretStopped = false;   // Enable the turret during autonomous
      DrivetrainSubsystem.forceCorrectEncoderResets = true;
      autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.turretSubsystem.isTurretStopped = true;      // Disable the turret when teleop begins
    robotContainer.turretSubsystem.homingPosition = 0;
    DrivetrainSubsystem.forceCorrectEncoderResets = false;
    robotContainer.teleopInitCommands().schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
