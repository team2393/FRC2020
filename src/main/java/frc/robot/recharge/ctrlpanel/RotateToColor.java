/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to rotate wheel based on camera info */
public class RotateToColor extends CommandBase
{
  private final ControlWheel wheel;

  /** The color that we should find,
   *  or -1 if we don't know where to go.
   */
  private int desired_color;

  private boolean is_finished;

  /** @param wheel Control wheel to turn */
  public RotateToColor(final ControlWheel wheel)
  {
    this.wheel = wheel;
    addRequirements(wheel);
  }

  @Override
  public void initialize()
  {
    // Determine which color we should go to
    desired_color = getDesiredColor();
    if (desired_color < 0)
      is_finished = true;
    else
    {
      wheel.fast();
      is_finished = false;
    }
  }
  
  private int getDesiredColor()
  {
    final String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() < 1)
      return -1;
    if (gameData.charAt(0) == 'B')
      return 0;
    if (gameData.charAt(0) == 'G')
      return 1;
    if (gameData.charAt(0) == 'R')
      return 2;
    if (gameData.charAt(0) == 'Y')
      return 3;
    return -1;
  }

  @Override
  public void execute()
  {
    if (desired_color < 0)
      return;

    // Did the camera detect a color?
    int color = wheel.getColor();
    if (color < 0)
    {
      // System.out.println("Unknown color");
      // Go slow to improve chance of catching that color
      wheel.slow();
      return;
    }

    // Have we reached the expected color?
    if (color == desired_color)
    {
      is_finished = true;
      System.out.println("Found " + ControlWheel.COLORS[color]);
    }
    else
      wheel.fast();
  }

  @Override
  public void end(final boolean interrupted)
  {
    wheel.spin(0);
  }

  @Override
  public boolean isFinished()
  {
    return is_finished;
  }
}
