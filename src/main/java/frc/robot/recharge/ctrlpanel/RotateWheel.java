/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to rotate wheel N times */
public class RotateWheel extends CommandBase
{
  private final ControlWheel wheel;
  private final int required_sectors;

  /** Number of color wheel sectors that we need to see go by */
  private int sectors;

  private int next_color = -1;

  /** @param wheel Control wheel to turn
   *  @param times How many times should be turn the wheel?
   */
  public RotateWheel(final ControlWheel wheel, final int times)
  {
    this.wheel = wheel;
    required_sectors = times * 8;
    addRequirements(wheel);
  }

  @Override
  public void initialize()
  {
    sectors = required_sectors;
    next_color = -1;
    wheel.fast();
  }

  @Override
  public void execute()
  {
    // Did the camera detect a color?
    int color = wheel.getColor();
    if (color < 0)
    {
      // System.out.println("Unknown color");
      // Go slow to improve chance of catching that color
      wheel.slow();
      return;
    }
    // Got the color, move on
    wheel.fast();

    // Is this the first time we see a color?
    if (next_color < 0)
    {
      next_color = (color + 1) % ColorDetector.COLORS.length;
      System.out.println("Started on " + ColorDetector.COLORS[color] +
                         ", looking for " + ColorDetector.COLORS[next_color]);
      return;
    }

    // Have we reached the next expected color?
    if (color == next_color)
    {
      // One more sector done
      --sectors;
      if (isFinished())
      {
        System.out.println("Found " + ColorDetector.COLORS[color] +
                            ", methinks I'm DONE!");
        return;
      }
      next_color = (color + 1) % ColorDetector.COLORS.length;
      System.out.println("Found " + ColorDetector.COLORS[color] +
                         ", now looking for " + ColorDetector.COLORS[next_color] +
                         ", " + sectors + " more sectors");
    }
  }

  @Override
  public void end(final boolean interrupted)
  {
    wheel.spin(0);
  }

  @Override
  public boolean isFinished()
  {
    return sectors <= 0;
  }
}
