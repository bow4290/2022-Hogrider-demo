package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Flags;

public class ElevatorSubsystem extends SubsystemBase {
  private WPI_TalonFX elevatorClimbMotor = new WPI_TalonFX(ElevatorConstants.elevatorClimbMotorChannel);
  private final DoubleSolenoid climberSolenoid;

  public enum TravArmStatus { UP, DOWN }
  public static TravArmStatus travArmStatus;
  public ElevatorSubsystem() {
    if (!Flags.elevator) throw new Error("Elevator flag must be set to create an ElevatorSubsystem!");
    elevatorClimbMotor.setNeutralMode(NeutralMode.Brake);
    elevatorClimbMotor.setInverted(TalonFXInvertType.CounterClockwise);

    elevatorClimbMotor.configForwardSoftLimitEnable(true);
    elevatorClimbMotor.configReverseSoftLimitEnable(true);
    elevatorClimbMotor.configForwardSoftLimitThreshold(ElevatorConstants.forwardSoftLimit);
    elevatorClimbMotor.configReverseSoftLimitThreshold(ElevatorConstants.reverseSoftLimit);

    elevatorClimbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    elevatorClimbMotor.setSelectedSensorPosition(0);
    
    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ElevatorConstants.armUpChannel, ElevatorConstants.armDownChannel);
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

  public void retractArm() {
    climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    travArmStatus = TravArmStatus.UP;
  }

  public void extendArm() {
    climberSolenoid.set(DoubleSolenoid.Value.kForward);
    travArmStatus = TravArmStatus.DOWN;
  }

  public double getEncoderposition(){
    return elevatorClimbMotor.getSelectedSensorPosition();
  }

  public static TravArmStatus getTravArmStatus(){
    return travArmStatus;
  }

  @Override
  public void periodic() {
  }
}
