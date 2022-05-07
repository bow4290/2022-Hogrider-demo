package frc.robot.commands.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DefaultHoodCommand extends CommandBase {
  private Limelight limelight;
  private HoodSubsystem hoodSubsystem;
  private TurretSubsystem turretSubsystem;
  
  public DefaultHoodCommand(Limelight limelight, HoodSubsystem hoodSubsystem, TurretSubsystem turretSubsystem) {
    this.limelight = limelight;
    this.hoodSubsystem = hoodSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (turretSubsystem.isTurretReady()) {
      if ((limelight.getDistance() > HoodConstants.hoodExtendDistance) && !hoodSubsystem.isHoodExtended()) {
        hoodSubsystem.extendHoodSolenoid();
      } else if ((limelight.getDistance() < HoodConstants.hoodExtendDistance) && hoodSubsystem.isHoodExtended()) {
        hoodSubsystem.retractHoodSolenoid();
      }
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
