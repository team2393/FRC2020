/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Drive train (them wheels) */
public class DriveTrain extends SubsystemBase
{
  // Switch to TalonFX?
  private final TalonFX left_main = new TalonFX(RobotMap.LEFT_MOTOR_MAIN);
  // private final TalonFX right_main = new TalonFX(RobotMap.RIGHT_MOTOR_MAIN);
  // private final TalonFX left_slave = new TalonFX(RobotMap.LEFT_MOTOR_SLAVE);


  public DriveTrain()
  {
    // Calling any CTRE CAN bus API will add the Phoenix diagnostics server to the robot
    // --> Dummy access to CAN bus even if we don't have a motor, yet
    System.out.println("Left main Talon firmware: " + Integer.toHexString(left_main.getFirmwareVersion()));
  }

  public void drive(double speed)
  {
    left_main.set(ControlMode.PercentOutput, speed);
  }
}