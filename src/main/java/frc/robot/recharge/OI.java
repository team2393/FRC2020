/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Operator Interface Definitions
 * 
 * One place to find which button does what
 */
public class OI
{
    public static final XboxController JOYSTICK = new XboxController(0);

    public static final JoystickButton DEMO_BUTTON = new JoystickButton(JOYSTICK, XboxController.Button.kA.value);

}