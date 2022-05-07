package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  public static WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.shooterMotorChannel);

  private double targetSpeed = 0;
  private double manualRPM = ShooterConstants.manualShooterSpeedRPM;
  private double actualSpeed = 0;
  private double errorRatio = 0;
  private int thresholdLoops = 0;
  private int goodThresholdLoopCount = 10;

  public ShooterSubsystem() {
    shooterMotor.configFactoryDefault();
    shooterMotor.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configVoltageCompSaturation(ShooterConstants.shooterMotorVoltage);
    shooterMotor.enableVoltageCompensation(true);
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    shooterMotor.configOpenloopRamp(0.3);

    shooterMotor.config_kF(0, ShooterConstants.kF);
    shooterMotor.config_kP(0, ShooterConstants.kP);
    shooterMotor.config_kI(0, ShooterConstants.kI);
    shooterMotor.config_kD(0, ShooterConstants.kD);

    // SmartDashboard.putNumber("Manual Shooter Speed ", manualRPM);
    // SmartDashboard.putNumber("kP", 0.0);
    // SmartDashboard.putNumber("kI", ShooterConstants.kI);
    // SmartDashboard.putNumber("kD", 0.0);
  }

  /* SparkMAX motor controllers require a weird velocity unit.
   * To make this simple, let's provide an RPM value and allow the software to convert it.
   *
   *               Pulses
   * VelocityRaw = ------
   *               100 ms
   *
   *               Pulses     1000 ms     60 sec       1 rev
   * VelocityRPM = ------  *  -------  *  ------  *  -----------
   *               100 ms      1 sec      1 min      2048 pulses
   * 
   * VelocityRPM = VelocityControlMode * 600 / 2048
   * VelocityRaw = VelocityRPM * 2048 / 600
  */
  public void shoot(double shooterRPM) { 
    targetSpeed = shooterRPM*2048/600;
    shooterMotor.set(ControlMode.Velocity, targetSpeed);
  }

  public void manualShoot() {
    // Obtain the manual shooter speed from the dashboard. If it can't, default to the ShooterConstants value.
    manualRPM = SmartDashboard.getNumber("Manual Shooter Speed ", ShooterConstants.manualShooterSpeedRPM);

    targetSpeed = (manualRPM*2048)/600;
    shooterMotor.set(ControlMode.Velocity, targetSpeed);
  }

  private double getShooterVelocityRaw() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  public double getShooterRPM() {
    return getShooterVelocityRaw()*600/2048;
  }

  public boolean isShooterReady() {
    return thresholdLoops > goodThresholdLoopCount;
  }

  @Override
  public void periodic() {
    actualSpeed = getShooterVelocityRaw();
    errorRatio = actualSpeed/targetSpeed;

    if (errorRatio > 0.98 && errorRatio < 1.02) { // was 1.03
      ++thresholdLoops;
    } else {
      thresholdLoops = 0;
    }

    //SmartDashboard.putNumber("Shooter Commanded Speed", targetSpeed);
    // SmartDashboard.putNumber("Shooter Actual Speed", getShooterRPM());
    // SmartDashboard.putBoolean("Is Shooter Ready? ", isShooterReady());
    
    // shooterMotor.config_kP(0, SmartDashboard.getNumber("kP", 0.0));
    // shooterMotor.config_kI(0, SmartDashboard.getNumber("kI", 0.0001));
    // shooterMotor.config_kD(0, SmartDashboard.getNumber("kD", 0.0));
  }
}
