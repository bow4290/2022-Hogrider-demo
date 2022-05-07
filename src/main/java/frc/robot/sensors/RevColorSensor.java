/*
    Proximity thresholds should be in the range [0-2047]
*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class RevColorSensor{
  private final ColorSensorV3 colorSensor;

  private double redLowThresh;
  private double redHighThresh;
  private double greenLowThresh;
  private double greenHighThresh;
  private double blueLowThresh;
  private double blueHighThresh;
  private int proxLowThresh;
  private int proxHighThresh;

  public RevColorSensor(double redLowThresh, double redHighThresh, double greenLowThresh, double greenHighThresh,
                        double blueLowThresh, double blueHighThresh, int proxLowThresh, int proxHighThresh, Boolean i2c) {
    this.redLowThresh    = redLowThresh;
    this.redHighThresh   = redHighThresh;
    this.greenLowThresh  = greenLowThresh;
    this.greenHighThresh = greenHighThresh;
    this.blueLowThresh   = blueLowThresh;
    this.blueHighThresh  = blueHighThresh;
    this.proxLowThresh   = proxLowThresh;
    this.proxHighThresh  = proxHighThresh;

    colorSensor = new ColorSensorV3(i2c ? I2C.Port.kOnboard : I2C.Port.kMXP);
  }

  public boolean isTarget() {
    final Color color = colorSensor.getColor();
    return (isRedThresholdMet(color) && isBlueThresholdMet(color));
  }
  //checking for green threshold and proximity were removed

  public boolean isRedThresholdMet(Color color) {
    return (color.red > redLowThresh && color.red < redHighThresh);
  }

  public boolean isGreenThresholdMet(Color color) {
    return (color.green > greenLowThresh && color.green < greenHighThresh);
  }

  public boolean isBlueThresholdMet(Color color) {
    return (color.blue > blueLowThresh && color.blue < blueHighThresh);
  }

  public boolean isProximityMet() {
    final int proximity = getProximity();
    return (proximity > proxLowThresh && proximity < proxHighThresh);
  }

  public double getRed() { return colorSensor.getColor().red; }

  public double getGreen() { return colorSensor.getColor().green; }

  public double getBlue() { return colorSensor.getColor().blue; }

  public double getIR() { return colorSensor.getIR(); }

  public int getProximity() { return colorSensor.getProximity(); }


}
