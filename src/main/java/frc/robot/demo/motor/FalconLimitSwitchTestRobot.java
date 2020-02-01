/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;

/** Example of limit switch usage */
public class FalconLimitSwitchTestRobot extends BasicRobot
{
  private final XboxController joystick = new XboxController(0);
  // Works the same for WPI_TalonSRX or WPI_TalonFX
  private final WPI_TalonFX motor = new WPI_TalonFX(1);
    
  @Override
  public void robotInit()
  {
    super.robotInit();

    // Configure motor
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);

    // This is the default configuration:
    // Motor will run until one connects a limit switch and closes it.
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Alternatively, can select "Deactivated" source, or configure to be "NormallyClosed",
    // which is better for a fail-safe setup.
    // TODO Test if motor stops when limit switch is hit
    // TODO Change to NC
    // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
  }
  
  @Override
  public void teleopPeriodic()
  {
    // TODO Allow moving beyond limit switch
    // motor.overrideLimitSwitchesEnable(joystick.getAButton());

    // Manually control motor speed via joystick
    motor.set(ControlMode.PercentOutput, joystick.getY(Hand.kRight));
    
    // Show limit switch state.
    // This should always work, no matter if switch is configured to do anything
    SmartDashboard.putBoolean("Limit Switch", motor.isFwdLimitSwitchClosed() == 1);
  }
}