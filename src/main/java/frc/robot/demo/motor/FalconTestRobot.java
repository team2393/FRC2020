/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.BasicRobot;

/** Example of PID control for position
 * 
 *  Uses the Falcon's encoder
 *  with PID control performed in the RoboRIO
 */
public class FalconTestRobot extends BasicRobot
{
  private final XboxController joystick = new XboxController(0);
  private final WPI_TalonFX motor = new WPI_TalonFX(1);

  private double desired_position = 0;

  /** Controller which computes error, correction etc. */
  private final PIDController position_pid = new PIDController(0.0005, 0, 0.0001);
  
  /** Command that reads current position,
   *  invokes PIDController to get output value,
   *  feeds that to the motor
   */
  private final PIDCommand hold_position = new PIDCommand(
    position_pid,
    () -> motor.getSelectedSensorPosition(),
    () -> desired_position,
    output -> motor.set(ControlMode.PercentOutput, output));
    
  @Override
  public void robotInit()
  {
    super.robotInit();
    // Configure motor
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(2.0);

    // Allow setting PID parameters on dashboard
    SmartDashboard.putData("Position PID", position_pid);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Position", motor.getSelectedSensorPosition());
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
    hold_position.schedule();
  }
  
  @Override
  public void autonomousPeriodic()
  {
    // Toggle between two desired positions every 2 seconds
    final int steps_per_rev = 2048;
    desired_position = ((System.currentTimeMillis() / 2000) % 2) * steps_per_rev * 1.0;
    SmartDashboard.putNumber("Position Error", position_pid.getPositionError());
    SmartDashboard.putBoolean("At Position", position_pid.atSetpoint());
  }

  @Override
  public void teleopPeriodic()
  {
    // Manually control motor speed via joystick
    motor.set(ControlMode.PercentOutput, joystick.getY(Hand.kRight));
  }
}