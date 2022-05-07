package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Flags;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.Shooter.DefaultShootHighCommand;
import frc.robot.commands.Shooter.ShootDiscard;
import frc.robot.commands.Shooter.ShootHigh;
import frc.robot.commands.Shooter.ShootManual;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Auto.AutoDriveForDistanceCommand;
import frc.robot.commands.Auto.AutoTurnLeftAngleCommand;
import frc.robot.commands.Auto.AutoTurnRightAngleCommand;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.*;
import frc.robot.sensors.BallIdentification;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.RevColorSensor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Turret.DisableTurretCommand;
import frc.robot.commands.Turret.EnableTurretCommand;
import frc.robot.commands.Turret.ToggleTurretCommand;
import frc.robot.commands.Turret.TurretCommand;

public class RobotContainer {
  public static Joystick joystickLeft = new Joystick(JoystickConstants.LEFT_JOYSTICK);
  public static Joystick joystickRight = new Joystick(JoystickConstants.RIGHT_JOYSTICK);
  // public static Joystick xboxController = new Joystick(JoystickConstants.XBOX_CONTROLLER);
  public static XboxController xboxController = new XboxController(JoystickConstants.XBOX_CONTROLLER);

  private IndexerSubsystem indexerSubsystem;
  public DrivetrainSubsystem drivetrainSubsystem;
  public TurretSubsystem turretSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  public Limelight limelight = new Limelight();

  public RevColorSensor redBallColorSensorI2C;
  public RevColorSensor blueBallColorSensorI2C;
  public RevColorSensor redBallColorSensorMXP;
  public RevColorSensor blueBallColorSensorMXP;

  public BallIdentification ballUpper;
  public BallIdentification ballLower;

  public RobotContainer() {
    if (Flags.drivetrain) {
      drivetrainSubsystem = new DrivetrainSubsystem();
      drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(() -> -joystickLeft.getY(), () -> -joystickRight.getY(), drivetrainSubsystem));
    }

    if (Flags.intake) {
      intakeSubsystem = new IntakeSubsystem();
      intakeSubsystem.setDefaultCommand(new DefaultIntakeInCommand(() -> xboxController.getLeftTriggerAxis(), intakeSubsystem));
    }

    shooterSubsystem = new ShooterSubsystem();


    if (Flags.indexer) {
      // Color values are from 0.00 - 1.00 (0% to 100% of the measured color).
      BallIdentification.Threshold red = new BallIdentification.Threshold(0.4, 1.0, 0.0, 0.18);
      BallIdentification.Threshold blueHigh = new BallIdentification.Threshold(0.0, 0.3, 0.2, 1.0);
      BallIdentification.Threshold blueLow = new BallIdentification.Threshold(0.0, 0.3, 0.26, 1.0);
      ballUpper = new BallIdentification(red, blueHigh, false);
      ballLower = new BallIdentification(red, blueLow, true);
      indexerSubsystem = new IndexerSubsystem();
      indexerSubsystem.setDefaultCommand(new DefaultIndexerCommand(indexerSubsystem, shooterSubsystem, intakeSubsystem, ballUpper, ballLower));
    }

    if (Flags.turret) {
      turretSubsystem = new TurretSubsystem(limelight);
      turretSubsystem.setDefaultCommand(new TurretCommand(limelight, turretSubsystem));
    }

    if (Flags.elevator) elevatorSubsystem = new ElevatorSubsystem();

    shooterSubsystem.setDefaultCommand(new DefaultShootHighCommand(() -> xboxController.getRightTriggerAxis(), limelight, shooterSubsystem, turretSubsystem));

    configureButtonBindings();
  }

  /* Xbox Controller Button Bindings:
     Buttons:
      1 - A           6 - RightBump
      2 - B           7 - Back
      3 - X           8 - Start
      4 - Y           9 - LeftStickIn
      5 - LeftBump   10 - RightStickIn

     Axes:
      0 - LeftX       3 - RightTrig
      1 - LeftY       4 - RightX
      2 - LeftTrig    5 - RightY
  */
  private void configureButtonBindings() {
    // setJoystickButtonWhenHeld(xboxController, 5, new ManualTurretClockwiseCommand(turretSubsystem));
    // setJoystickButtonWhenHeld(xboxController, 6, new ManualTurretCounterClockwiseCommand(turretSubsystem));

    if (Flags.drivetrain) {
      setJoystickButtonWhenPressed(joystickLeft, 1, new ShiftGearDown(drivetrainSubsystem));
      setJoystickButtonWhenPressed(joystickRight, 1, new ShiftGearUp(drivetrainSubsystem));
    }

    if (Flags.intake) {
      // setXboxControllerButtonWhenHeld(xboxController, 5, new IntakeIn(intakeSubsystem));
      setXboxControllerButtonWhenHeld(xboxController, 5, new IntakeToggle(intakeSubsystem));
    }

    if (Flags.indexer) {
      setXboxControllerButtonWhenHeld(xboxController, 9, new ManualIndexerCommand(indexerSubsystem));
      setXboxControllerButtonWhenHeld(xboxController, 10, new ReverseIndexerCommand(indexerSubsystem));
    }

    setXboxControllerButtonWhenHeld(xboxController, 6, new ToggleTurretCommand(turretSubsystem));

    setXboxControllerButtonWhenHeld(xboxController, 4, new ElevatorUpCommand(elevatorSubsystem));
    setXboxControllerButtonWhenHeld(xboxController, 1, new ElevatorDownCommand(elevatorSubsystem));
    setXboxControllerButtonWhenHeld(xboxController, 3, new ArmToggle(elevatorSubsystem));

    setXboxControllerButtonWhenHeld(xboxController, 2, new ShootManual(shooterSubsystem));
    // setXboxControllerButtonWhenHeld(xboxController, 6, new ShootHigh(limelight, shooterSubsystem, turretSubsystem));
  }

  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }

  /*
   * WhileHeld starts the command. If the command ends whilst the button is still pressed,
   * the command is rescheduled. The command is cancelled when button is released.
   */
  // private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
  //   new JoystickButton(joystick, button).whileHeld(command);
  // }

  /**
   * WhenHeld starts the command. If the command ends whilst the button is still pressed,
   * the command is not rescheduled. The command is cancelled when button is released.
   */
  private void setJoystickButtonWhenHeld(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenHeld(command);
  }

  private void setXboxControllerButtonWhenHeld(XboxController xboxController, int button, CommandBase command) {
    new JoystickButton(xboxController, button).whenHeld(command);
  }

  public void periodic() {
    ballLower.update();
    ballUpper.update();
  }
}
