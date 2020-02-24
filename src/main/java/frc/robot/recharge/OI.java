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

  /** Reset joystick memory */
  public static void reset()
  {
    // The 'getXXXPressed' methods remember if a button was
    // pressed, even just briefly.
    // Trouble is this includes times when it was pressed while
    // we were disabled, and then suddenly things start to happen
    // as we enable..
    // --> Read each 'pressed' state once to clear it
    for (int i=0; i<=10; ++i)
      joystick.getRawButtonPressed(i);
  }

  public static boolean selectDriveMode()
  {
    return joystick.getRawButton(XboxController.Button.kBack.value);
  }

  public static boolean selectClimbMode()
  {
    return joystick.getRawButton(XboxController.Button.kStart.value);
  }


  public static boolean isLowGearRequested()
  {
    return joystick.getTriggerAxis(Hand.kRight) > .5;
  }

  public static boolean isHighGearRequested()
  {
    return joystick.getBumperPressed(Hand.kRight);
  }

  public static boolean force_low_speed = false;

  private static double getSpeedFactor()
  {
    if (force_low_speed  ||  joystick.getTriggerAxis(Hand.kRight) > 0.6)
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

  public static final boolean isToggleHeadingholdPressed()
  {
      return joystick.getXButtonPressed();
  }

  public static final boolean isAlignOnTargetHeld()
  {
      return joystick.getYButton();
  }

  public static boolean isIntakeDownRequested()
  {
    return joystick.getTriggerAxis(Hand.kLeft) > 0.5;
  }

  public static boolean isIntakeUpRequested()
  {
    return joystick.getBumperPressed(Hand.kLeft);
  }

  public static final boolean isShootHeld()
  {
    return joystick.getAButton();
  }

  /** @return Up/Down for hood */
  public static double getHoodSpeed()
  {
    if (!joystick.getStickButton(Hand.kRight))
      return 0;
    else
      return square(joystick.getX(Hand.kRight));
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
  
  /** @return telescope speed -1 (down) to 1 (up) */
  public static double getTelescopeSpeed()
  {
    // Triggers return 0..1
    // Right trigger for 'up', left trigger 'down'
    return joystick.getTriggerAxis(Hand.kRight) -
           joystick.getTriggerAxis(Hand.kLeft);
  }
  
  public static double getClimbSpeed()
  {
    // Holding one bumber to climb, both to climb faster
    double speed = 0.0;
    if (joystick.getBumper(Hand.kLeft))
      speed += 0.3;
    if (joystick.getBumper(Hand.kRight))  
      speed += 0.3;
    return speed;  
  }   
}