/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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
  // Must have encoder (angle) and limit switch (end position)
  private final WPI_TalonFX hood_motor = new WPI_TalonFX(RobotMap.HOOD_MOTOR);
  
  // PID
  private final PIDController pid = new PIDController(0, 0, 0);

  /** Desired arm/rotator angle. Negative to disable PID */
  private double desired_angle = -1;

  public Hood()
  {
    PowerCellAccelerator.commonSettings(hood_motor, NeutralMode.Brake);

    // Encoder for position (angle)
    hood_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Limit switch at end position
    hood_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }
  
  /** Move hood towards 'home' limit switch.
   *  @return true if at 'home' position
   */
  public boolean homeHood()
  {
    // TODO Find voltage for slow movement towards home switch
    // TODO Is home switch the forward or reverse limit switch?
    hood_motor.setVoltage(-0.1);
    final boolean homed = hood_motor.isFwdLimitSwitchClosed() == 1;
    if (homed)
     hood_motor.setSelectedSensorPosition(0);
    return homed;
  }

  public double getHoodAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle
    // TODO Try frc-characterization of 'arm'
    // An angle of zero (degrees/radians) should be 'horizontal'
    // in case we want to use ArmFeedforward
    return 1.0 * hood_motor.getSelectedSensorPosition();
  }

  /** @param speed Directly set motor speed for testing */
  public void setAngleMotor(final double speed)
  {
    // Disable automated control
    desired_angle = -1;
    hood_motor.set(speed);
  }

  public PIDController getPID()
  {
    return pid;
  }

  /** @param angle Set angle for PID-controlled angle rotator, negative to disable */
  public void setHoodAngle(final double angle)
  {
    desired_angle = angle;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());

    if (desired_angle >= 0)
    {
      final double correction = pid.calculate(getHoodAngle(), desired_angle);
      hood_motor.setVoltage(correction);
    }
  }
}
