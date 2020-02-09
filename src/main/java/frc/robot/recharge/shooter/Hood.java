/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Ejector hood
 * 
 *  Angle of rotatable hood/shield/deflector adjusts
 *  the angle at which balls are ejected.
 */
public class Hood extends SubsystemBase
{
  // Motors
  // Must have encoder (angle)
  private final WPI_TalonFX angle_adjustment = new WPI_TalonFX(RobotMap.ANGLE_MOTOR);
  
  // PID
  // TODO Tune, then turn into ProfiledPIDController or use ProfiledPIDSubsystem
  private final PIDController shooter_angle_pid = new PIDController(0, 0, 0);

  public Hood()
  {
    PowerCellAccelerator.commonSettings(angle_adjustment, NeutralMode.Brake);

    // Encoder for position
    angle_adjustment.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }
  
  private double getHoodAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle
    // TODO Try frc-characterization of 'arm'
    // An angle of zero (degrees/radians) must be 'horizontal'
    // because  ArmFeedforward  uses cos(angle) to determine impact of gravity,
    // which is at maximum for angle 0 (cos(0)=1) and vanishes at 90 deg (cos(90)=0)

    return angle_adjustment.getSelectedSensorPosition();
  }

  public void setHoodAngle(final double angle)
  {
    // Calculate angle with encoder values and use PID to adjust

    // TODO First connect joystick axis to 'angle' to simply move motor
    angle_adjustment.setVoltage(angle);

    // Calibrate angle
    // Then change this to
    // shooter_angle_pid.setSetpoint(angle);
    // enable PID in periodic() below and tune PID.
    // Then change to ProfiledPIDController
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());

    // TODO See setHoodAngle(angle);
    // angle_adjustment.setVoltage(shooter_angle_pid.calculate(getHoodAngle()));
  }
}
