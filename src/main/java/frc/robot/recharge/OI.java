/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

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

  public static final JoystickButton extend_control_wheel = new JoystickButton(joystick, XboxController.Button.kBumperLeft.value);
  public static final JoystickButton retract_control_wheel = new JoystickButton(joystick, XboxController.Button.kBumperRight.value);
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

  /** 'Signed square' to get more sensitivity around joystick center
   *  
   *  Same idea as in DifferentialDrive.arcadeDrive,
   *  but drive train passes the speed & rotation values
   *  on un-squared to keep PID-based moves linear.
   *  Instead, we square the joystick values so that interactive
   *  drive is de-sensitized.
   */
  private static double square(final double value)
  {
    return Math.signum(value) * (value * value);
  }

  /** @return Speed (1=full ahead) */
  public static double getSpeed()
  {
    double speed = square(getSpeedFactor() * -joystick.getY(Hand.kLeft));
    // speed = speed_limiter.calculate(speed);
    return speed;
  }

  /** @return Left/right steering */
  public static double getDirection()
  {
    if (joystick.getStickButton(Hand.kRight))
      return 0;
    else
      return square(getSpeedFactor() * joystick.getX(Hand.kRight));
  }

  /** @return Up/Down for hood */
  public static double getHoodSpeed()
  {
    if (!joystick.getStickButton(Hand.kRight))
      return 0;
    else
      return square(joystick.getX(Hand.kRight));
  }

  public static final boolean isToggleHeadingholdPressed()
  {
      return joystick.getRawButtonPressed(XboxController.Button.kY.value);
  }

  public static final boolean isAlignOnTargetHeld()
  {
      return joystick.getRawButton(XboxController.Button.kA.value);
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