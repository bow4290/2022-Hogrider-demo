package frc.robot;

public final class Constants {
  public static final class JoystickConstants {
    public static final int LEFT_JOYSTICK   = 0;
    public static final int RIGHT_JOYSTICK  = 1;
    public static final int XBOX_CONTROLLER = 2;
  }

  public static final class DriveConstants {
    // Drive Motor Channels
    public static final int rightMotor1Channel = 5;
    public static final int rightMotor2Channel = 6;
    public static final int leftMotor1Channel  = 7;
    public static final int leftMotor2Channel  = 8;
 
    // Drive Pneumatic Channels
    public static final int gearShiftUpChannel   = 4;
    public static final int gearShiftDownChannel = 5;

    // Drive Gear Ratio Values
    private static final double stage1Ratio     = 60/12;
    private static final double stage2Ratio     = 28/28;
    private static final double lowSpreadRatio  = 44/22;
    private static final double highSpreadRatio = 34/32;

    // Drive Calculations
    // inches
    private static final double wheelDiam   = 3.75;
    // inches
    private static final double wheelCircum = wheelDiam*Math.PI;
    // inches
    private static final double driveWidth  = 25.5;
    // Only for "zero-point" turns
    private static final double driveCircum = driveWidth *Math.PI;

    // Drive Distance Calculations
    private static final int encoderCPR = 2048;
    private static final double lowCountsPerWheelRev  = stage1Ratio * stage2Ratio * lowSpreadRatio * encoderCPR;
    private static final double highCountsPerWheelRev = stage1Ratio * stage2Ratio * highSpreadRatio * encoderCPR;
    // inches per 1 encoder count (low gear)
    public static final double encoderLowDistanceConversion  = wheelCircum/lowCountsPerWheelRev;
    // inches per 1 encoder count (high gear)
    public static final double encoderHighDistanceConversion = wheelCircum/highCountsPerWheelRev;
    
    // To turn x degrees, run turn command until encoderCounts >= (driveCountsPerDeg*x).
    // counts per 1 degrees (low gear)
    public static final double driveLowCountsPerDeg  = driveCircum/encoderLowDistanceConversion/360;
    // counts per 1 degrees (high gear)
    public static final double driveHighCountsPerDeg = driveCircum/encoderHighDistanceConversion/360;

    // Auto Drive Speeds
    public static final double autonomousDriveSpeed = 0.6;
    public static final double autonomousTurnSpeed  = 0.5;
  }

  public static final class ElevatorConstants {
    public static int elevatorClimbMotorChannel = 10;

    // Revolutions the motor can rotate CW
    public static double forwardSoftLimit = 175000;
    // Revolutions the motor can rotate CCW
    public static double reverseSoftLimit = 5000;

    // -1 to 1 (Percent Output)
    public static double elevatorSpeed = 0.90;

    public static int armUpChannel = 6;
    public static int armDownChannel = 7;
  }

  public static final class HoodConstants {
    public static int hoodSolenoidExtendChannel  = 3;
    public static int hoodSolenoidRetractChannel = 2;

    // Distance from target in inches where hood will extend
    public static double hoodExtendDistance = 160;
  }

  public static final class IndexerConstants {
    public static int upperIndexMotorChannel = 0;
    public static int lowerIndexMotorChannel = 1;
    
    // was 0.5
    public static double upperShootingIndexSpeed = 0.4;
    // was 0.275
    public static double lowerShootingIndexSpeed = 0.225;
    public static double upperIntakingIndexSpeed = 0.2;
    public static double lowerIntakingIndexSpeed = 0.275;
    public static double reverseIndexSpeed = 0.25;
  }

  public static final class IntakeConstants {
    public static int intakeMotorChannel = 13;

    public static int intakeUpChannel = 0;
    public static int intakeDownChannel = 1;

    public static double intakeTriggerbuffer = 0.05;
  }

  public static final class ShooterConstants {
    public static int shooterMotorChannel = 2;
    public static double shooterMotorVoltage = 11.0;

    public static double manualShooterSpeedRPM = 1500;
    public static double discardSpeedRPM = 1500.0;
    // a(x) + b
    public static double EquationAdjustA = 0;
    // a(x) + b
    public static double EquationAdjustB = 0;

    //1023.0/20330.0
    public static double kF = 1023.0/19900.0;
    public static double kP = 0.0;
    // was 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double shooterTriggerbuffer = 0.05;
  }

  public static final class TurretConstants {
    public static int deviceID = 11;

    // Soft limit to prevent turret from turning too far
    public static int forwardRotations = 95;
    // Soft limit to prevent turret from turning too far
    public static int reverseRotations = -95;

    public static double manualTurnSpeed    = 0.3;
    // This variable is useless (see turretKP comment)
    public static double defaultTrackSpeed  = 0.3;
    // This variable is actually less than the track speed (see above comment)
    // used to be -0.55
    public static double defaultSearchSpeed = -0.7;
    // NOTE: This variable is not for homing to a target, it's for going to 0 pos
    public static double turretHomingSpeed  = 0.6;

    public static double aimOffsetDistance = 2;

    // NOTE: This variable overrides defaultTrackSpeed
    public static double turretKP = 0.1;
  }

  public static final class LimelightConstants {
    /* To calculate the limelight mount angle (a1):
     * 1) Place the robot at a known distance
     * 2) Measure limelight lense height (h1) and target height (h2)
     * 3) Calculate: a1 = atan((h2-h1)/d)-yError
    */
    // Distance from ground to limelight lense
    public static final double h1 = 29.75;
    // Distance from ground to target (8'8")
    public static final double h2 = 104;
    // Limelight mount angle (0 = facing forward, 90 = facing the ceiling)
    public static final double a1 = 35.3;
  }
}
