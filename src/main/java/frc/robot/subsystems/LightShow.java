package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class LightShow extends SubsystemBase {

  public static final class Constants {
    public static final int leftPixelCount = 64;
    public static final int rightPixelCount = 64;
    public static final int totalPixels = leftPixelCount + rightPixelCount;
  }

  // NOTE: can only allocate ONE strip of LEDs (in series). This is a limitation
  // of how WPILib implements the addressable LED strips.
  // https://www.chiefdelphi.com/t/can-not-allocate-a-second-addressableled-pwm-port/376859

  AddressableLED leds = new AddressableLED(CANMapping.lightsPWM);
  AddressableLEDBuffer pixels = new AddressableLEDBuffer(Constants.totalPixels);

  String previousBuffer = pixels.toString();
  Timer timer = new Timer();

  public LightShow() {
    leds.setLength(Constants.totalPixels);
    leds.start();
    timer.start();
  }

  public void disable() {
    fillPercentage(0, 0, 0);
  }

  public void greengohappy() {
    fillPercentage(0, 50, 0);
  }

  public void redgoboom() {
      fillPercentage(50, 0, 0);
    }

  public void fillPercentage(int redPercent, int greenPercent, int bluePercent) {
    for (int i = 0; i < Constants.totalPixels; i++) {
      pixels.setRGB(i, redPercent, greenPercent, bluePercent);
    }
  }

  @Override
  public void periodic() {
    var currentBuffer = pixels.toString();
    var needsUpdate = previousBuffer != currentBuffer;
    if (needsUpdate) {
      leds.setData(pixels);
      previousBuffer = currentBuffer;
    }
  }

}
