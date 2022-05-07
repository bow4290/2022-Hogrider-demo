package frc.robot.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

public class BallIdentification implements Sendable {
  private final ColorSensorV3 sensor;

  private Color ballColor = getAllianceColor();

  public static enum Color { RED, BLUE, NEITHER }

  // TODO: why does this need to be a static class
  public static class Threshold {
    public final double redLow, redHigh, blueLow, blueHigh;

    public Threshold(double redLow, double redHigh,  double blueLow, double blueHigh) {
      this.redLow = redLow;
      this.redHigh = redHigh;
      this.blueLow = blueLow;
      this.blueHigh = blueHigh;
    }

    public Boolean check(edu.wpi.first.wpilibj.util.Color color) {
      return (color.red > redLow && color.red < redHigh) && (color.blue > blueLow && color.blue < blueHigh);
    }
  }

  private final Threshold redThresh, blueThresh;

  public BallIdentification(Threshold red, Threshold blue, Boolean i2c) {
    sensor = new ColorSensorV3(i2c ? I2C.Port.kOnboard : I2C.Port.kMXP);
    redThresh = red;
    blueThresh = blue;
  }

  private edu.wpi.first.wpilibj.util.Color cachedColor;
  public void update() {
    cachedColor = sensor.getColor();
  }

  // If the sensor senses a red ball or a blue ball, then a ball is present.
  public boolean isBallPresent() {
    return (redThresh.check(cachedColor) || blueThresh.check(cachedColor));
  }

  public Color getBallColor() {
    if (redThresh.check(cachedColor) && !blueThresh.check(cachedColor)) {
      ballColor = Color.RED;
    } else if (blueThresh.check(cachedColor) && !redThresh.check(cachedColor)) {
      ballColor = Color.BLUE;
    }
    return ballColor;
  }

  public static Color getAllianceColor() {
    switch (DriverStation.getAlliance()) {
      case Red:
        return Color.RED;
      case Blue:
        return Color.BLUE;
      case Invalid:
        return Color.NEITHER;
      default:
        return Color.NEITHER;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Is there a ball?", this :: isBallPresent, null);
  }
}
