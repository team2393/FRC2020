/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Operator Interface Definitions
 * 
 * One place to find which button does what
 */
public class OI
{
  public static final XboxController joystick = new XboxController(0);

  public static final JoystickButton enable_wheel = new JoystickButton(joystick, XboxController.Button.kA.value);
  public static final JoystickButton autorotate_wheel = new JoystickButton(joystick, XboxController.Button.kB.value);
  public static final JoystickButton rotate_to_color = new JoystickButton(joystick, XboxController.Button.kX.value);
  
  
  public static final Trigger shift_low = new Trigger(() -> joystick.getTriggerAxis(Hand.kRight) > .5);
  public static final JoystickButton shift_high = new JoystickButton(joystick, XboxController.Button.kBumperRight.value);
  
  private static double getSpeedFactor()
  {
    if (joystick.getTriggerAxis(Hand.kRight) > 0.6)
      return 0.5;
    else
      return 1;
  }

  // TODO Wait 1/4 second to reach full speed? Same for rotation?
  // private static final SlewRateLimiter speed_limiter = new SlewRateLimiter(4);

  /** @return Speed (1=full ahead) */
  public static double getSpeed()
  {
    double speed = getSpeedFactor() * -joystick.getY(Hand.kLeft);
    // speed = speed_limiter.calculate(speed);
    return speed;
  }

  /** @return Left/right steering */
  public static double getDirection()
  {
    return getSpeedFactor() * joystick.getX(Hand.kRight);
  }

  public static final boolean isToggleHeadingholdPressed()
  {
      return joystick.getRawButtonPressed(XboxController.Button.kA.value);
  }

  /** @return Manual fortune wheel speed */
  public static double getWheelSpeed()
  {
    // Each trigger returns value 0..1
    // Combine them into -1 .. 1 range
    double value = - joystick.getTriggerAxis(Hand.kLeft) + joystick.getTriggerAxis(Hand.kRight);
    // Slow down
    return value/3;
  }
}