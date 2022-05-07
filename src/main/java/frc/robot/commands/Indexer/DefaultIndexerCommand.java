package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.sensors.BallIdentification;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultIndexerCommand extends CommandBase {
  private IndexerSubsystem indexerSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private BallIdentification ballUpper;
  private BallIdentification ballLower;

  public DefaultIndexerCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, 
                               IntakeSubsystem intakeSubsystem, BallIdentification ballUpper, 
                               BallIdentification ballLower) {
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ballUpper = ballUpper;
    this.ballLower = ballLower;

    addRequirements(indexerSubsystem);
  }

@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(shooterSubsystem.isShooterReady()) {
      // When shooting, turn both motors at their specified shoot speeds.
      indexerSubsystem.turnBothIndexMotors(IndexerConstants.upperShootingIndexSpeed, IndexerConstants.lowerShootingIndexSpeed);
    } else if (intakeSubsystem.isIntakeSpinning()) {
      if (!ballUpper.isBallPresent()) {
        // If no ball is present at the upper sensor, turn both motors until the ball is present at the upper sensor.
        indexerSubsystem.turnBothIndexMotors(IndexerConstants.upperIntakingIndexSpeed, IndexerConstants.lowerIntakingIndexSpeed);
      } else if (ballUpper.isBallPresent() && !ballLower.isBallPresent()) {
        // If ball is at upper ssensor and not lower sensor, stop upper motor and turn lower motor until ball is at the lower sensor.
        indexerSubsystem.turnBothIndexMotors(0, IndexerConstants.lowerIntakingIndexSpeed);
      } else {
        indexerSubsystem.turnBothIndexMotors(0, 0);
      }
    } else {
      // If not shooting and not intaking, turn motors off.
      indexerSubsystem.turnBothIndexMotors(0, 0);
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
