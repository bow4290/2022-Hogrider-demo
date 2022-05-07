package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class ReverseIndexerCommand extends CommandBase {
  private IndexerSubsystem indexerSubsystem;

  public ReverseIndexerCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    indexerSubsystem.turnBothIndexMotors(-IndexerConstants.reverseIndexSpeed, -IndexerConstants.reverseIndexSpeed);
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
