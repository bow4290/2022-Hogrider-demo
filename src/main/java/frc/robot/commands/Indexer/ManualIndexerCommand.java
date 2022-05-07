package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class ManualIndexerCommand extends CommandBase {
  private IndexerSubsystem indexerSubsystem;

  public ManualIndexerCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    indexerSubsystem.turnBothIndexMotors(0.5, 0.5);
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
