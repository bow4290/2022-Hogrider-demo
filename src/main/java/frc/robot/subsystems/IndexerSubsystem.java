package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flags;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private WPI_VictorSPX upperIndexMotor;
  private WPI_VictorSPX lowerIndexMotor;

  public IndexerSubsystem(){
    if (!Flags.indexer) throw new Error("Indexer flag must be set to create an IndexerSubsystem!");
    
    upperIndexMotor = new WPI_VictorSPX(IndexerConstants.upperIndexMotorChannel);
    lowerIndexMotor = new WPI_VictorSPX(IndexerConstants.lowerIndexMotorChannel);

    upperIndexMotor.setInverted(true);
    lowerIndexMotor.setInverted(true);

    upperIndexMotor.configVoltageCompSaturation(11);
    lowerIndexMotor.configVoltageCompSaturation(11);
    upperIndexMotor.enableVoltageCompensation(true);
    lowerIndexMotor.enableVoltageCompensation(true);
  }

  public void turnBothIndexMotors(double upperIndexSpeed, double lowerIndexSpeed){
    turnUpperIndexMotor(upperIndexSpeed);
    turnLowerIndexMotor(lowerIndexSpeed);
  }

  public void turnUpperIndexMotor(double upperIndexSpeed){
    upperIndexMotor.set(ControlMode.PercentOutput, upperIndexSpeed);
  }

  public void turnLowerIndexMotor(double lowerIndexSpeed){
    lowerIndexMotor.set(ControlMode.PercentOutput, lowerIndexSpeed);
  }

  @Override
  public void periodic() {
  }
}
