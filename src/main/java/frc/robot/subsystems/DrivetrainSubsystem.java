package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private WPI_TalonFX leftMotor1 = new WPI_TalonFX(DriveConstants.leftMotor1Channel);
  private WPI_TalonFX leftMotor2 = new WPI_TalonFX(DriveConstants.leftMotor2Channel);
  private WPI_TalonFX rightMotor1 = new WPI_TalonFX(DriveConstants.rightMotor1Channel);
  private WPI_TalonFX rightMotor2 = new WPI_TalonFX(DriveConstants.rightMotor2Channel);

  private final DifferentialDrive drivetrain;

  private final DoubleSolenoid gearShiftSolenoid;
  public enum GearShiftPosition { UP, DOWN }
  private static GearShiftPosition gearShiftPosition;

  public DrivetrainSubsystem() {
    leftMotor1.setInverted(TalonFXInvertType.CounterClockwise);
    leftMotor2.follow(leftMotor1);
    leftMotor2.setInverted(InvertType.FollowMaster);

    rightMotor1.setInverted(TalonFXInvertType.Clockwise);
    rightMotor2.follow(rightMotor1);
    rightMotor2.setInverted(InvertType.FollowMaster);

    // Make the drivetrain brake when the motors are set to 0.
    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);

    // Smooth the motor inputs to regulate power draw.
    leftMotor1.configOpenloopRamp(0.6);
    leftMotor2.configOpenloopRamp(0.6);
    rightMotor1.configOpenloopRamp(0.6);
    rightMotor2.configOpenloopRamp(0.6);

    // Don't draw more than 11 volts. Keeps speed consistent even when battery output isn't.
    leftMotor1.configVoltageCompSaturation(11);
    leftMotor2.configVoltageCompSaturation(11);
    rightMotor1.configVoltageCompSaturation(11);
    rightMotor2.configVoltageCompSaturation(11);

    leftMotor1.enableVoltageCompensation(true);
    leftMotor2.enableVoltageCompensation(true);
    rightMotor1.enableVoltageCompensation(true);
    rightMotor2.enableVoltageCompensation(true);

    leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    drivetrain = new DifferentialDrive(leftMotor1, rightMotor1);

    gearShiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.gearShiftUpChannel, DriveConstants.gearShiftDownChannel);
    gearShiftPosition = null;
  }

  public void drive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed, true);
  }

  public void resetDriveEncoders() {
    leftMotor1.setSelectedSensorPosition(0);
    rightMotor1.setSelectedSensorPosition(0);
  }

  public GearShiftPosition getGearShiftPosition() {
    return gearShiftPosition;
  }

  public void shiftDown() {
    // kForward is DOWN
    gearShiftSolenoid.set(DoubleSolenoid.Value.kForward);
    gearShiftPosition = GearShiftPosition.DOWN;
  }

  public void shiftUp() {
    // kReverse is UP
    gearShiftSolenoid.set(DoubleSolenoid.Value.kReverse);
    gearShiftPosition = GearShiftPosition.UP;
  }

  public double getLeftCalculatedPosition() {
    if(gearShiftPosition == GearShiftPosition.DOWN) {
      return (getLeftRawEncoderPosition() * (DriveConstants.encoderLowDistanceConversion));
    } else {
      return (getLeftRawEncoderPosition() * (DriveConstants.encoderHighDistanceConversion));
    }
  }

  public double getRightCalculatedPosition() {
    if(gearShiftPosition == GearShiftPosition.DOWN) {
      return (getRightRawEncoderPosition() * (DriveConstants.encoderLowDistanceConversion));
    } else {
      return (getRightRawEncoderPosition() * (DriveConstants.encoderHighDistanceConversion));
    }
  }

  public double getTurnLeftDegrees(){
    if(gearShiftPosition == GearShiftPosition.DOWN) {
      return (getRightRawEncoderPosition() / (DriveConstants.driveLowCountsPerDeg));
    } else {
      return (getRightRawEncoderPosition() / (DriveConstants.driveHighCountsPerDeg));
    }
  }

  public double getTurnRightDegrees(){
    if(gearShiftPosition == GearShiftPosition.DOWN) {
      return (getLeftRawEncoderPosition() / (DriveConstants.driveLowCountsPerDeg));
    } else {
      return (getLeftRawEncoderPosition() / (DriveConstants.driveHighCountsPerDeg));
    }
  }

  private double getLeftRawEncoderPosition() {
    return leftMotor1.getSelectedSensorPosition();
  }

  private double getRightRawEncoderPosition() {
    return rightMotor1.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if (gearShiftPosition == GearShiftPosition.DOWN){
      SmartDashboard.putString("Drive GEAR ", "LOW");
    } else {
      SmartDashboard.putString("Drive GEAR ", "HIGH");
    }
  }
}
