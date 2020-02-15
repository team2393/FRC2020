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
  // Power Distribution Panel
  //
  // 40 Amp connectors
  // 1) Drive motor
  // 2) Drive motor
  // 3) Drive motor
  // 4) Drive motor
  // 5) Shooter motor
  // 6) Lift-up climber  motor
  // 7) Intake wheel motor
  // 8) Intake raise/lower motor 1
  //
  // Below 40 Amp ports
  // 1) Horizontal conveyor motor
  // 2) Vertical conveyor motor
  // 3) Ejection angle hood motor
  // 4) Control panel color wheel motor
  // 5) Telescope raising motor
  // 6) Camera LED ring
  // 7) Intake raise/lower motor 2
  // 8) 3rd prox sensor
  //
  // PDP controller port -> RoboRIO
  // PDP PCM port -> PCM, compressor, solenoids
  //
  // PDP VRM port ->
  // VRM 12V, 2A
  // 1) Radio
  // 2) must not be used
  //
  // VRM 12V, 500mA
  // 1) Prox sensor at end of conveyors
  // 2) Prox sensor in 'ejector'
  //
  // VRM 5V, 500mA
  // 1)
  // 2)
  //
  // VRM 5V, 2A
  // 1) Raspberry Pi power
  // 2) Color LED strip power

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
  public final static int INTAKE_SPINNER = 8;
  public final static int INTAKE_ROTATOR = 9;
  public final static int INTAKE_ROTATOR_SLAVE = 10;
  public final static int ANGLE_MOTOR = 11;

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
