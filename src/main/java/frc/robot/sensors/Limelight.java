package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;

public class Limelight {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static double avgYError = 0.0;

  public static enum LedMode { ledPipeline, ledOff, ledBlink, ledOn }

  public static enum CamMode { vision, driving }

  public Limelight() {
  }

  /** Set the state of the LED */
  public void setLedMode(LedMode mode) { table.getEntry("ledMode").setNumber(mode.ordinal()); }

  public LedMode getLedMode() {
    int mode_num = table.getEntry("ledMode").getNumber(0).intValue();

    if (mode_num == 0) {
      return LedMode.ledPipeline;
    } else if (mode_num == 1) {
      return LedMode.ledOff;
    } else if (mode_num == 1) {
      return LedMode.ledBlink;
    } else if (mode_num == 2) {
      return LedMode.ledOn;
    }

    return LedMode.ledOff;
  }

  /** Set the camera mode (vision processing or driving camera) */
  public void setCamMode(CamMode mode) { table.getEntry("camMode").setNumber(mode.ordinal()); }

  /** Set the pipeline number (0-9) */
  public void setPipeline(int pipeline) { table.getEntry("pipeline").setNumber(pipeline); }

  /** Whether the limelight has any valid targets (0 or 1) */
  public double getTarget() { return table.getEntry("tv").getDouble(0); }

  /** Whether the limelight has any valid targets (0 or 1) */
  public boolean isTarget() { return table.getEntry("tv").getDouble(0) == 1; }

  /** Horizontal offset from limelight crosshair to target crosshair */
  public double getXError() { return table.getEntry("tx").getDouble(0.00); }

  /** Horizontal offset from limelight crosshair to target crosshair with user-specified offset distance in inches */
  public double getXErrorWithOffset(double targetOffsetDistance) {
    return getXError() + Math.toDegrees(Math.atan2(targetOffsetDistance, getDistance()));
  }

  /** Vertical offset from limelight crosshair to target crosshair */
  public double getYError() { return table.getEntry("ty").getDouble(0.00); }

  /** Vertical offset from limelight crosshair to target crosshair with user-specified offset distance */
  public double getYErrorWithOffset(double targetOffsetDistance) {
    return getYError() + Math.toDegrees(Math.atan2(targetOffsetDistance, getDistance()));
  }

  public double getAvgYError(double historic_weight) {
    avgYError = ((historic_weight * avgYError) + ((1.0 - historic_weight) * getYError())) / 2.0;
    return avgYError;
  }

  /** Target area (0% to 100% of image) */
  public double getArea() { return table.getEntry("ta").getDouble(0.00); }

  /** Skew or rotation (-90 deg to 0 deg) */
  public double getSkew() { return table.getEntry("ts").getDouble(0.00); }

  /** Pipeline latency contribution (ms) */
  public double getLatency() { return table.getEntry("tl").getDouble(0.00); }

  /** Sidelength of shortest side of fitted box (pixels) */
  public double getShortLength() { return table.getEntry("tshort").getDouble(0); }

  /** Sidelength of longest side of fitted box (pixels) */
  public double getLongLength() { return table.getEntry("tlong").getDouble(0); }

  /** Horizontal sidelength of rough bouding box (0-320 pixels) */
  public double getHorizontalLength() { return table.getEntry("thor").getDouble(0); }

  /** Vertical sidelength of rough bouding box (0-320 pixels) */
  public double getVerticalLength() { return table.getEntry("tvert").getDouble(0); }

  /** True active pipeline of the camera (0-9) */
  public double getPipeline() { return table.getEntry("getpipe").getDouble(0); }

  /** Horizontal distance from limelight to target */
  public double getDistance() {
    return (LimelightConstants.h2 - LimelightConstants.h1) /
        Math.tan(Math.toRadians(LimelightConstants.a1 + getYError()));
  }
}
