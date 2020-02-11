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
import frc.robot.recharge.RobotMap;

/** RPM-controlled spinner */
public class Spinner
{
  private final WPI_TalonFX motor = new WPI_TalonFX(RobotMap.SHOOTER_MOTOR);

  /** TODO Encoder ticks for one turn of the wheel */
  private final static double TICK_PER_REVOLUTION = 3310;
  
  // FF & PID for shooter motor to set RPM
  // https://trickingrockstothink.com/blog_posts/2019/10/19/tuning_pid.html

  /** TODO Feed-forward velocity constant: Volts per RPM */
  private double kV = 0.00239;

  /** TODO P gain */
  private final PIDController pid = new PIDController(0.01, 0, 0);

  public Spinner()
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralMode.Coast);

    // Encoder for speed
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }  
  
  public void configure(double kV, double P)
  {
    this.kV = kV;
    pid.setP(P);
  }

  public void setVoltage(final double volt) 
  {
    motor.setVoltage(volt);  
  }  

  public double getRPM()
  {
    // Sensor gives ticks per 100ms, i.e. 10 times that per second.
    // Turn into revolutions, scale to revs per minute.
    return motor.getSelectedSensorVelocity() * 10.0 / TICK_PER_REVOLUTION * 60.0;
  }  

  public void setRPM(final double desired_rpm)
  {
    final double feed_forward = desired_rpm*kV; 
    final double voltage = feed_forward + pid.calculate(getRPM(), desired_rpm);
    setVoltage(voltage);
  }
}
