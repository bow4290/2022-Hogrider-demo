package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.sensors.BallIdentification;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoIndexerCommand extends CommandBase {
  private IndexerSubsystem indexerSubsystem;
  private BallIdentification ballUpper;
  private BallIdentification ballLower;
  private ShooterSubsystem shooterSubsystem;

  public AutoIndexerCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, BallIdentification ballUpper, BallIdentification ballLower) {
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
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

    } else {
      if (!ballUpper.isBallPresent()) {
        indexerSubsystem.turnBothIndexMotors(IndexerConstants.upperIntakingIndexSpeed, IndexerConstants.lowerIntakingIndexSpeed);
        // If no ball is present at the upper sensor, turn both motors until the ball is present at the upper sensor.
      
      } else if (ballUpper.isBallPresent() && !ballLower.isBallPresent()) {
        indexerSubsystem.turnBothIndexMotors(0, IndexerConstants.lowerIntakingIndexSpeed);
        // If ball is at upper ssensor and not lower sensor, stop upper motor and turn lower motor until ball is at the lower sensor.

      } else {
        indexerSubsystem.turnBothIndexMotors(0, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.turnBothIndexMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
