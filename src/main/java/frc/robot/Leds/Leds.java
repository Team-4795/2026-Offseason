package frc.robot.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Leds.Leds.BlinkState;

public class Leds {
  public enum BlinkState {
    SOLID,
    FAST,
    SlOW;
  }

  private final int PORT = 0;

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private final int LED_LENGTH = 59;

  private int frame;

  private boolean teamColorsAnimation = false;

  private Color color1 = Color.kBlack;
  private Color color2 = Color.kBlack;
  private BlinkState blink = BlinkState.SOLID;

  private static Leds instance;

  public static Leds getInstance() {
    if (instance != null) {
      return instance;
    }
    throw new Error("Leds get instance no work D:");
  }

  public Leds() {
    led = new AddressableLED(PORT);
    buffer = new AddressableLEDBuffer(LED_LENGTH);
  }
  //Section 1 high set
  //Section 2 medium set
  //Section 3 low set
  //Section 4 other set


  public void setOneColor(int r, int g, int b, int section) {
    for (int i = (section-1)*LED_LENGTH/4; i < (section)*LED_LENGTH/4; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }
}
