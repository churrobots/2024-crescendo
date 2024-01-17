package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightShow extends SubsystemBase {
  private final int klightPWM = 9;

  // NOTE: you can only allocate ONE strip of LEDs (in series). This is a
  // limitation
  // of how WPILib implements the addressable LED strips.
  // https://www.chiefdelphi.com/t/can-not-allocate-a-second-addressableled-pwm-port/376859

  // Our lights are set up as two 8x32 grids in series.
  AddressableLED leds = new AddressableLED(klightPWM);
  final int ROWS = 8;
  final int COLS = 32;
  final int STRIPS = 2;
  final int PIXELS = (ROWS * COLS) * STRIPS;
  AddressableLEDBuffer pixels = new AddressableLEDBuffer(PIXELS);
  Timer t = new Timer();
  double waitTime = 0.0;
  private final Intake m_Intake;

  public LightShow(Intake m_Intake) {
    leds.setLength(PIXELS);
    leds.start();
    t.start();
    this.m_Intake = m_Intake;
  }

  public int getPixelCount() {
    return PIXELS;
  }

  public void clearPixels() {
    fill(0, 0, 0);
  }

  public void fill(int red, int green, int blue) {
    for (int i = 0; i < PIXELS; i++) {
      pixels.setRGB(i, red, green, blue);
    }
    leds.setData(pixels);
  }

  public void fillPercentage(int redPercent, int greenPercent, int bluePercent) {
    for (int i = 0; i < PIXELS; i++) {
      pixels.setRGB(i, redPercent, greenPercent, bluePercent);
    }
    leds.setData(pixels);
  }

  public void setPixels(AddressableLEDBuffer buffer) {
    boolean isError = buffer.getLength() != PIXELS;
    if (isError) {
      fill(255, 0, 0);
      return;
    }
    // if (!priorBuffer.equals(buffer)) {
    leds.setData(buffer);
    // }
  }

  public void setPurple() {
    fillPercentage(5, 0, 5);
  }
  public void setBlue() {
    fillPercentage(0, 0, 5);
  }

  public void setGreen() {
    fillPercentage(0, 5, 0);
  }

  public void setRed() {
    fillPercentage(5, 0, 0);
  }

  @Override
  public void periodic() {
    // runDefaultLights();
    if (RobotState.isEStopped()) {
      setRed();
    } else if (RobotState.isAutonomous()) {
      setPurple();
    } else if (m_Intake.isYoinking()) {
      setGreen();
    } else if (m_Intake.isYeeting()) {
      setBlue();
    } else {
      turnOff();
    }
  }

  public void turnOff() {
    fill(0, 0, 0);
  }

  public void runDefaultLights() {
    turnOff();
  }
}
