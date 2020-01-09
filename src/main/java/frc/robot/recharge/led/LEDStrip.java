/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.RobotMap;

/**
 * Add your docs here.
 */
public class LEDStrip
{
  private static final int N = 30;
  private final AddressableLED strip = new AddressableLED(RobotMap.LED_STRIP);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(N);

  public LEDStrip()
  {
    strip.setLength(N);
    setAll(255, 0, 0);
    strip.start();
  }
  
  public void setAll(final int r, final int g, final int b)
  {
    for (int i=0; i<N; ++i)
      buffer.setRGB(i, r, g, b);
    strip.setData(buffer);
  }

  /** Millisecs for one rainbow cycle */
  private static final int rainbow_ms = 2000;

  /** Steps to advance hue for each 20 Hz update.
   *  One cycle means 180 degrees of hue.
   *  Split into rainbow_ms/1000 seconds,
   *  spread over 20 updates. 
   */
  private static final int rainbox_step = 180 / (rainbow_ms/1000) / 20;
  private int rainbox_start = 0;

  /** Cyling 'rainbow' pattern */
  public void rainbow()
  {
    for (int i=0; i<N; ++i)
      buffer.setHSV(i, (rainbox_start + (i * 180 / N)) % 180, 255, 128);
    rainbox_start = (rainbox_start + rainbox_step) % 180;
    
    strip.setData(buffer);
  }

  /** Number of LEDs used for the 'direction' indicator */
  private static final int pointer_size = 6;

  /** Indicate direction to a target
   *
   *  @param direction -1 .. 0 .. 1 for left .. center .. right
   */
  public void indicateDirection(final double direction)
  {
    // 'on target', close to 0: ----------GGGGG----------
    // full left, -1          : GGGGGRRRRRRRRRR----------
    // somewhat left, -0.5    : ---GGGGGRRRRRRR----------

    // Usable space for moving central pointer left or right
    final int usable_space = (N - pointer_size) / 2;
    final int start = N/2 + usable_space * MathUtil.clamp(direction, -1.0, 1.0);
    final int start = start + pointer_size;

    int i;
    // Overall 'background' color
    for (i=0; i<N; ++i)
      buffer.setRGB(i, 0, 0, 0);

    // Potential red region from center to start (direction > 0)
    for (i=N/2; i<start; ++i)
      buffer.setRGB(i, 255, 0, 0);

    // Green start..end section
    for (i=start; i<end; ++i)
      buffer.setRGB(i, 0, 255, 0);

    // Potential red region from right end to center (direction < 0)
    for (i=end; i<N/2; ++i)
      buffer.setRGB(i, 255, 0, 0);
  }
}
