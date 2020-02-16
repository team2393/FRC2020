/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.motor;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;

/** Example of PID control for position
 * 
 *  Uses the Falcon's encoder
 *  with PID control performed in the RoboRIO
 */
public class EncoderTestRobot extends BasicRobot
{
  private final WPI_TalonFX motor = new WPI_TalonFX(1);

  @Override
  public void robotInit()
  {
    super.robotInit();
    // Configure motor
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(0.5);

    // The default, integrated encoder gives 2048 ticks per revolution
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // The standalong CTRE mag encoder may support absolute,
    // but the built-in one always reports 0.
    // In any case, even if the absolute mode is supported,
    // for example with a TalonSRX controller connected to the mag encoder,
    // looks like the absolute encoder only knows its exact position within one revolution.
    // It won't track the absolute position for multiple full revolutions
    // across reboots.
    // motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Revs", motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("RPS", motor.getSelectedSensorVelocity()*10.0);
  }
}