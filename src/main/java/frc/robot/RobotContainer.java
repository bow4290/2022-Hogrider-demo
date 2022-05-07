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

  private Command AutoDriveCollectAndShoot2;
  private Command AutoDriveCollectAndShoot;
  private Command AutoDriveAndShoot;
  private Command AutoDriveAndCollect;
  private Command AutoDriveOnly;
  private Command AutoCrossCountry;
  private Command AutoFunkyFourBallOnly;
  private Command AutoNothing;
  private Command AutoPlop;
  private Command AutoTestingOnly;

  SendableChooser<Command> chooser = new SendableChooser<>();

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

    autoCommands();
    shooterSubsystem.setDefaultCommand(new DefaultShootHighCommand(() -> xboxController.getRightTriggerAxis(), limelight, shooterSubsystem, turretSubsystem));

    chooser.setDefaultOption("3-Ball High Auto", AutoDriveCollectAndShoot2);
    chooser.addOption("2-Ball High Auto", AutoDriveCollectAndShoot);
    chooser.addOption("1-Ball High Auto", AutoDriveAndShoot);
    chooser.addOption("1-Ball Low Auto", AutoPlop);
    chooser.addOption("Drive and Collect", AutoDriveAndCollect);
    chooser.addOption("Drive Only", AutoDriveOnly);
    // chooser.addOption("Cross Country", AutoCrossCountry);
    // chooser.addOption("4-Ball High Auto", AutoFunkyFourBallOnly);
    chooser.addOption("Do Nothing", AutoNothing);
    // chooser.addOption("TESTING ONLY", AutoTestingOnly);
    SmartDashboard.putData(chooser);

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

  private void autoCommands(){
    AutoPlop = 
    new SequentialCommandGroup(
      new ToggleTurretCommand(turretSubsystem),
      new ParallelRaceGroup(
          new ShootDiscard(shooterSubsystem),
          new WaitCommand(5)
        )
      // new AutoDriveForDistanceCommand(drivetrainSubsystem, -60)
      // it doesnt work like that :(
    );

    AutoDriveCollectAndShoot2 = 
      new SequentialCommandGroup(
        new ShiftGearDown(drivetrainSubsystem),
        new DisableTurretCommand(turretSubsystem),
        new IntakeDown(intakeSubsystem),
        new ShiftGearDown(drivetrainSubsystem),
        new ParallelRaceGroup(
          new AutoDriveForDistanceCommand(drivetrainSubsystem, 40),
          new IntakeIn(intakeSubsystem)
        ),
        new WaitCommand(0.20),
        new ParallelRaceGroup(
          new AutoTurnRightAngleCommand(drivetrainSubsystem, (90.0 + 35 + 30.0)),
          new IntakeIn(intakeSubsystem)
        ),
        new EnableTurretCommand(turretSubsystem),
        new WaitCommand(0.20),
        new ParallelRaceGroup(
          new ShootHigh(limelight, shooterSubsystem, turretSubsystem),
          new WaitCommand(3.25)
        ),
        new DisableTurretCommand(turretSubsystem),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 30.0),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new AutoDriveForDistanceCommand(drivetrainSubsystem, (117 + 6)),
                    new WaitCommand(0.20),
                    // (whatever (the + math)) + don't_care
                    new AutoTurnRightAngleCommand(drivetrainSubsystem, (210.0 - (90.0 + 22.5)) + 20.0),
                    new EnableTurretCommand(turretSubsystem)
                    ),
                new IntakeIn(intakeSubsystem)),
        new ParallelRaceGroup(
                        new ShootHigh(limelight, shooterSubsystem, turretSubsystem),
                        new WaitCommand(4)),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 120),
        new AutoDriveForDistanceCommand(drivetrainSubsystem, 13 * 12)
      );
    
    AutoDriveCollectAndShoot = 
    new SequentialCommandGroup(
      new ShiftGearDown(drivetrainSubsystem),
      new DisableTurretCommand(turretSubsystem),
      new IntakeDown(intakeSubsystem),    
      new ShiftGearDown(drivetrainSubsystem),
      new ParallelRaceGroup(
        new AutoDriveForDistanceCommand(drivetrainSubsystem, 40), // used to be 40, did not go quite far enough
        new IntakeIn(intakeSubsystem)
      ),
      new WaitCommand(0.20),
      new ParallelRaceGroup(
        new AutoTurnRightAngleCommand(drivetrainSubsystem, (150)),     
        new IntakeIn(intakeSubsystem)
      ),
      new EnableTurretCommand(turretSubsystem),
      new WaitCommand(0.20),
      new ParallelRaceGroup(
        new ShootHigh(limelight, shooterSubsystem, turretSubsystem),
        new WaitCommand(5)
      )
    );

    AutoDriveAndShoot =
      new SequentialCommandGroup(
        new ShiftGearDown(drivetrainSubsystem),
        new DisableTurretCommand(turretSubsystem),
        new AutoDriveForDistanceCommand(drivetrainSubsystem, 40),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 180),
        new EnableTurretCommand(turretSubsystem),
        new ParallelRaceGroup(
          new ShootHigh(limelight, shooterSubsystem, turretSubsystem),
          new WaitCommand(5)
        )
      );
    
    AutoDriveAndCollect =
      new SequentialCommandGroup(
        new ShiftGearDown(drivetrainSubsystem),
        new DisableTurretCommand(turretSubsystem),
        new IntakeDown(intakeSubsystem),
        new ParallelRaceGroup(
          new AutoDriveForDistanceCommand(drivetrainSubsystem, 40),
          new IntakeIn(intakeSubsystem)
        )
      );
    
    AutoDriveOnly =
      new SequentialCommandGroup(
        new ShiftGearDown(drivetrainSubsystem),
        new DisableTurretCommand(turretSubsystem),
        new AutoDriveForDistanceCommand(drivetrainSubsystem, 40)
      );

    AutoCrossCountry = new SequentialCommandGroup(new ShiftGearDown(drivetrainSubsystem),
        new DisableTurretCommand(turretSubsystem),
        new IntakeDown(intakeSubsystem),
        new ParallelRaceGroup(
            new AutoDriveForDistanceCommand(drivetrainSubsystem, 40),
            new IntakeIn(intakeSubsystem)),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 180),
        new EnableTurretCommand(turretSubsystem),
        new ParallelRaceGroup(new ShootHigh(limelight, shooterSubsystem, turretSubsystem), new WaitCommand(5.0)),
        new DisableTurretCommand(turretSubsystem),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 50),
        new ParallelCommandGroup(new SequentialCommandGroup(
            new ParallelRaceGroup(new AutoDriveForDistanceCommand(drivetrainSubsystem, 40),
                new IntakeIn(intakeSubsystem)),
            new AutoTurnRightAngleCommand(drivetrainSubsystem, 80)),
            new ShootDiscard(shooterSubsystem)
                .until(() -> ballUpper.getBallColor() == BallIdentification.getAllianceColor())),
        new EnableTurretCommand(turretSubsystem),
        new ParallelRaceGroup(new ShootHigh(limelight, shooterSubsystem, turretSubsystem), new WaitCommand(5.0)));

    AutoFunkyFourBallOnly =
      new SequentialCommandGroup(
        new ShiftGearDown(drivetrainSubsystem),
        new DisableTurretCommand(turretSubsystem),
        new IntakeDown(intakeSubsystem),
        new ParallelRaceGroup(
          new AutoDriveForDistanceCommand(drivetrainSubsystem, 40),
          new IntakeIn(intakeSubsystem)
        ),
        new WaitCommand(0.20),
        new EnableTurretCommand(turretSubsystem),
        new ParallelRaceGroup(
          new AutoTurnRightAngleCommand(drivetrainSubsystem, (90.0 + 35.0 + 25.0)),
          new IntakeIn(intakeSubsystem)
        ),
        new WaitCommand(0.20),
        new ParallelRaceGroup(
          new ShootHigh(limelight, shooterSubsystem, turretSubsystem),
          new WaitCommand(2.75)
        ),
        new DisableTurretCommand(turretSubsystem),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 25),
        new ParallelRaceGroup(
          new AutoDriveForDistanceCommand(drivetrainSubsystem, (117+6)),
          new IntakeIn(intakeSubsystem)
        ),
        new WaitCommand(0.20),
        new AutoTurnLeftAngleCommand(drivetrainSubsystem, 82.5),
        new ParallelRaceGroup(
          new AutoDriveForDistanceCommand(drivetrainSubsystem, 90),
          new IntakeIn(intakeSubsystem)
        )
        //new EnableTurretCommand(turretSubsystem),
        // new ParallelRaceGroup(
        //   new ShootHigh(limelight, shooterSubsystem, turretSubsystem),
        //   new WaitCommand(5)
        // )
      );
    
    AutoNothing = 
      null;

    AutoTestingOnly = 
    null;
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public Command teleopInitCommands(){
    return new ParallelCommandGroup(
            // new IntakeUp(inqtakeSubsystem),
            new ShiftGearDown(drivetrainSubsystem)
           );
  }

  public void periodic() {
    ballLower.update();
    ballUpper.update();
  }
}
