/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

  public void rainbow()
  {
    for (int i=0; i<N; ++i)
      buffer.setHSV(i, (rainbox_start + (i * 180 / N)) % 180, 255, 128);
    rainbox_start = (rainbox_start + rainbox_step) % 180;
    
    strip.setData(buffer);
  }
}
