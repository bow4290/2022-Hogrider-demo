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
      indexerSubsystem.turnBothIndexMotors(IndexerConstants.upperShootingIndexSpeed, IndexerConstants.lowerShootingIndexSpeed);
      // When shooting, turn both motors at their specified shoot speeds.

    } else if (intakeSubsystem.isIntakeSpinning()) {
      if (!ballUpper.isBallPresent()) {
        indexerSubsystem.turnBothIndexMotors(IndexerConstants.upperIntakingIndexSpeed, IndexerConstants.lowerIntakingIndexSpeed);
        // If no ball is present at the upper sensor, turn both motors until the ball is present at the upper sensor.
      
      } else if (ballUpper.isBallPresent() && !ballLower.isBallPresent()) {
        indexerSubsystem.turnBothIndexMotors(0, IndexerConstants.lowerIntakingIndexSpeed);
        // If ball is at upper ssensor and not lower sensor, stop upper motor and turn lower motor until ball is at the lower sensor.

      } else {
        indexerSubsystem.turnBothIndexMotors(0, 0);
      }
    } else {
      indexerSubsystem.turnBothIndexMotors(0, 0);     // If not shooting and not intaking, turn motors off.
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
