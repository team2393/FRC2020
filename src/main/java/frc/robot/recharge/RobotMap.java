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
  public final static int LEFT_MOTOR_MAIN = 1;
  public final static int RIGHT_MOTOR_MAIN = 2;
  public final static int LEFT_MOTOR_SLAVE = 3;
  public final static int RIGHT_MOTOR_SLAVE = 4;

  // Shooter Motors
  public final static int SHOOTER_MOTOR = 5;
  // TODO  public final static int SHOOTER_MOTOR_SLAVE = ;
  public final static int CONVEYOR_BOTTOM = 6;
  public final static int CONVEYOR_TOP = 7;
  public final static int INTAKE_MOTOR = 8;
  // TODO  public final static int INTAKE_MOTOR_SLAVE = 8;
  public final static int INTAKE_POSITION = 9;
  public final static int ANGLE_MOTOR = 10;

  // Digital IO Sensors
  public final static int SHOOTER_SENSOR_TOP = 1;
  public final static int SHOOTER_SENSOR_MID = 2;

  // Motor port used for wheel-of-fortune on control panel
  public final static int CONTROL_PANEL_WHEEL = 9;

  // PWM port for LED Strip
  public static final int LED_STRIP = 8;

  // PCM port used to solenoids
  public static final int GEAR_SOLENOID = 7;

}