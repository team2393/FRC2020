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

  public static final JoystickButton enable_wheel = new JoystickButton(joystick, XboxController.Button.kA.value);
  public static final JoystickButton autorotate_wheel = new JoystickButton(joystick, XboxController.Button.kB.value);
  public static final JoystickButton rotate_to_color = new JoystickButton(joystick, XboxController.Button.kX.value);

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