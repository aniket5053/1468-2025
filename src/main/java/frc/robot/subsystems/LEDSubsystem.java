package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  // Blinking speeds  - TA TODO: test blinking speeds
  private static final double BlinkingOffTime = 0.25;
  private static final double fastBlinkingOnTime = 0.25;
  private static final double slowBlinkingOnTime = 2.0 * fastBlinkingOnTime;

  private static final int LED_Length = 88;

  private final AddressableLED m_led = new AddressableLED(9);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED_Length);
  // private AddressableLEDBufferView m_DriverData = m_ledBuffer.createView(0, 43);
  // private AddressableLEDBufferView m_OperatorData = m_ledBuffer.createView(44, LED_Length - 1);
  // Our LED strip has a density of 30 LEDs per meter per AndyMark- TA TODO: Verify LED Density
  private static final Distance kLedSpacing = Meters.of(1 / 30.0);

  private Color orangeColor;

  public LEDSubsystem() {

    m_led.setColorOrder(ColorOrder.kRGB); // sets convention to RGB

    // more intialization
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    setOrangePattern();
  }

  @Override
  public void periodic() {

    m_led.setData(m_ledBuffer);
  }

  //   public void setOff() {
  //     LEDPattern black = LEDPattern.solid(Color.kBlack);
  //     black.applyTo(m_ledBuffer);
  //     m_led.stop();
  //   }

  //   public Command setLEDsOff(LEDPattern pattern) {
  //     return runOnce(() -> setOff());
  //   }

  public void setDriveActionStarted() {
    // m_led.start();
    LEDPattern color = LEDPattern.solid(Color.kRed);
    // color = color.blink((Seconds.of(slowBlinkingOnTime)), (Seconds.of(BlinkingOffTime)));
    color.applyTo(m_ledBuffer);
  }

  public Command setDriveCmdStarted() {
    return runOnce(() -> setDriveActionStarted());
  }

  public void setDriveActionFinished() {
    // m_led.start();
    LEDPattern color = LEDPattern.solid(Color.kLime);
    // color = color.breathe(Seconds.of(fastBlinkingSpeed));
    color.applyTo(m_ledBuffer);
  }

  public Command setDriveCmdFinished() {
    return runOnce(() -> setDriveActionFinished());
  }

  public void setOperatorActionStarted() {
    // m_led.start();
    LEDPattern color = LEDPattern.solid(Color.kRed);
    // color = color.breathe(Seconds.of(slowBlinkingSpeed));
    color.applyTo(m_ledBuffer);
  }

  public Command setOperatorCmdStarted() {
    return runOnce(() -> setOperatorActionStarted());
  }

  public void setOperatorActionFinished() {
    // m_led.start();
    LEDPattern color = LEDPattern.solid(Color.kLime);
    // color = color.breathe(Seconds.of(fastBlinkingSpeed));
    color.applyTo(m_ledBuffer);
  }

  public Command setOperatorCmdFinished() {
    return runOnce(() -> setOperatorActionFinished());
  }

  public void setOperatorShoot() {
    // m_led.start();
    LEDPattern color = LEDPattern.solid(Color.kWhite);
    // color = color.breathe(Seconds.of(fastBlinkingSpeed));
    color.applyTo(m_ledBuffer);
  }

  public Command setOperatorShootCmd() {
    return runOnce(() -> setOperatorShoot());
  }

  public void setOrangePattern() {
    // m_led.start();
    orangeColor = new Color(255, 111, 0);
    LEDPattern orange = LEDPattern.solid(orangeColor);
    LEDPattern blue = LEDPattern.solid(Color.kRed);
    // LEDPattern bluePattern = blue.breathe(Seconds.of(1));
    // LEDPattern combined = orange.blend(bluePattern);
    orange.applyTo(m_ledBuffer); // applys the "combined" color
  }

  public void setWhiteBlinking() {
    // m_led.start();
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    // white = white.breathe(Seconds.of(fastBlinkingSpeed));
    white.applyTo(m_ledBuffer);
  }

  public void setGreenBlinking() {
    // m_led.start();
    LEDPattern green = LEDPattern.solid(Color.kLime);
    // green = green.breathe(Seconds.of(fastBlinkingSpeed));
    green.applyTo(m_ledBuffer);
  }

  public void setRedBlinking() {
    // m_led.start();
    LEDPattern red = LEDPattern.solid(Color.kRed);
    //    LEDPattern blinkingRed = red.breathe(Seconds.of(fastBlinkingOnTime));
    red.applyTo(m_ledBuffer);
  }

  // public void setRainbow() {
  //   // m_led.start();
  //   // Create an LED pattern that will display a rainbow across
  //   // all hues at maximum saturation and half brightness
  //   LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  //   // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
  // speed
  //   // of 1 meter per second.
  //   LEDPattern m_scrollingRainbow =
  //       m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  //   // Update the buffer with the rainbow animation
  //   m_scrollingRainbow.applyTo(m_ledBuffer);
  // }
}
