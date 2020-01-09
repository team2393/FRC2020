/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge;

/** Hardware Mappings
 *  
 *  One place to find what's connected how
 */ 
public class RobotMap
{    
  // Talon CAN IDs =================================
  // Drivetrain motors
  public final static int LEFT_MOTOR_SLAVE = 4;
  public final static int LEFT_MOTOR_MAIN = 2;
  public final static int RIGHT_MOTOR_SLAVE = 3;
  public final static int RIGHT_MOTOR_MAIN = 1;

  // Motor port used for wheel-of-fortune on control panel
  public final static int CONTROL_PANEL_WHEEL = 9;

  // PWM port for LED Strip
  public static final int LED_STRIP = 0;
}
