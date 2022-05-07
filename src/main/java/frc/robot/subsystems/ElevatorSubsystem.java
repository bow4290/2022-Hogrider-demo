package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private WPI_TalonFX elevatorClimbMotor = new WPI_TalonFX(ElevatorConstants.elevatorClimbMotorChannel);

  public ElevatorSubsystem() {
    elevatorClimbMotor.setNeutralMode(NeutralMode.Brake);
    elevatorClimbMotor.setInverted(TalonFXInvertType.CounterClockwise);

    elevatorClimbMotor.configForwardSoftLimitEnable(true);
    elevatorClimbMotor.configReverseSoftLimitEnable(true);
    elevatorClimbMotor.configForwardSoftLimitThreshold(ElevatorConstants.forwardSoftLimit);
    elevatorClimbMotor.configReverseSoftLimitThreshold(ElevatorConstants.reverseSoftLimit);

    elevatorClimbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    elevatorClimbMotor.setSelectedSensorPosition(0);
  }

  public void extendElevator(double climbSpeed){
    elevatorClimbMotor.set(ControlMode.PercentOutput, ElevatorConstants.elevatorSpeed);
  }

  public void retractElevator(double climbSpeed){
    elevatorClimbMotor.set(ControlMode.PercentOutput, -ElevatorConstants.elevatorSpeed);
  }

  public void stopElevator(){
    elevatorClimbMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getEncoderposition(){
    return elevatorClimbMotor.getSelectedSensorPosition();
  }
}
