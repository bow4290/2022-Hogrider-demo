package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flags;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final DoubleSolenoid intakeSolenoid;

  public enum IntakeStatus { DOWN, UP }
  public static IntakeStatus intakeStatus;

  public IntakeSubsystem() {
    if (!Flags.intake) throw new Error("Intake flag must be set to create an IntakeSubsystem!");
    
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorChannel, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(11);
    intakeMotor.setOpenLoopRampRate(0.5);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.intakeUpChannel, IntakeConstants.intakeDownChannel);
    intakeStatus = IntakeStatus.UP;
  }

  public void retractIntake() {
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    intakeStatus = IntakeStatus.UP;
  }

  public void extendIntake() {
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    intakeStatus = IntakeStatus.DOWN;
  }

  public void intakeSpin(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public boolean isIntakeSpinning(){
    return Math.abs(intakeMotor.get()) > 0 ? true : false;
  }

  public static IntakeStatus getIntakePosition() {
    return intakeStatus;
  }

  @Override
  public void periodic() {
    if (intakeStatus == IntakeStatus.DOWN){
      SmartDashboard.putString("Intake Status: ", "Extended");
    } else {
      SmartDashboard.putString("Intake Status: ", "Retracted");
    }
  }
}
