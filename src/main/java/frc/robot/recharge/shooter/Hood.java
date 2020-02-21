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
 * 
 *  Angle is calibrated by resetting encoder at startup,
 *  i.e. hood must be all the way 'down' when powered on.
 */
public class Hood extends SubsystemBase
{
  // Motor, must have encoder (angle)
  private final WPI_TalonFX hood_motor = new WPI_TalonFX(RobotMap.HOOD_MOTOR);
  
  // PID
  private final PIDController pid = new PIDController(0, 0, 0);

  /** Desired angle. Negative to disable PID */
  private double desired_angle = -1;

  public Hood()
  {
    PowerCellAccelerator.commonSettings(hood_motor, NeutralMode.Brake);

    // Encoder for position (angle)
    hood_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    hood_motor.setInverted(false);

    // Reset to zero on startup
    hood_motor.setSelectedSensorPosition(0);
  }
  
  /** @return Hood angle, degrees. 0 for horizontal, towards 90 for 'up' */
  public double getHoodAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle
    // TODO Try frc-characterization of 'arm'

    // Falcon encoder sends 2048 ticks per revolution
    final double encoder_angle = 360.0 / 2048.0;
    // Gears & chain results in hood moving slower than motor
    final double gearing = 18.0/42.0;

    // An angle of zero (degrees/radians) should be 'horizontal'
    //   90 deg = 'up'
    // ~120 deg = start position, fully retracted
    final double offset = 0.0;
    return offset + hood_motor.getSelectedSensorPosition() * encoder_angle * gearing;
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
