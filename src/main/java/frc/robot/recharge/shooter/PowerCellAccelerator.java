/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Conveyor and ejector
 * 
 *  'Fuel cell' balls from from hopper onto horizontal conveyor belt.
 *  Horizontal belt moves them to vertical conveyor,
 *  which feeds them to ejector/shooter.
 */
public class PowerCellAccelerator extends SubsystemBase 
{
  // Motors
  // TODO figure out what type of motor controllers will actually be used -- Tony was leaning towards falcons for most
  // Must have encoder (speed)
  private final WPI_TalonFX shooting_motor = new WPI_TalonFX(RobotMap.SHOOTER_MOTOR);
  private final WPI_TalonFX conveyor_top = new WPI_TalonFX(RobotMap.CONVEYOR_TOP);
  private final WPI_TalonFX conveyor_bottom = new WPI_TalonFX(RobotMap.CONVEYOR_BOTTOM); 
  
  // Sensors
  private final DigitalInput shooter_sensor_mid = new DigitalInput(RobotMap.SHOOTER_SENSOR_MID);
  private final DigitalInput shooter_sensor_top = new DigitalInput(RobotMap.SHOOTER_SENSOR_TOP);

  public final static double CONVEYOR_VOLTAGE = 5.0;

  public final static double SHOOTER_VOLTAGE = 10.5;

  public final static double MINIMUM_SHOOTER_RPM = 5000;

  private final Timer keep_running_timer = new Timer();
  private boolean shoot = false;

  public PowerCellAccelerator()
  {
    commonSettings(shooting_motor, NeutralMode.Coast);
    commonSettings(conveyor_top, NeutralMode.Brake);
    commonSettings(conveyor_bottom, NeutralMode.Brake);

    // Encoder for speed
    shooting_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }
  
  /** @param motor Motor to configure with common settings
   *  @param mode Neutral mode
   */
  static void commonSettings(final BaseMotorController motor, final NeutralMode mode)
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(mode);
  }
  
  public void moveConveyor(final double volt)
  {
    // Should the conveyor have the ability to move backwards or should it only be "on" or "off"
    // Conveyors should probably be moved seperately
    conveyor_bottom.setVoltage(volt);
    conveyor_top.setVoltage(volt);
  }
  
  /** Returns true if power cell is in "ready" position at end of vertical conveyor */
  public boolean powerCellReady()
  {
    return !shooter_sensor_mid.get();
  }

  /** Turn shooter 'on' or 'off'.
   * 
   *  When turned 'off', it actually remains
   *  running for a few seconds so in case
   *  we want to turn it 'on' again it's
   *  already up to speed.
   */
  public void eject(final boolean on_off)
  {
    // If shooter was on and is now requested off,
    // start timer so it keeps running for a little longer
    if (shoot == true  &&  on_off == false)
      keep_running_timer.start();
    
    shoot = on_off;
  }

  public void setShooterVoltage(final double volt) 
  {
    shooting_motor.setVoltage(volt);  
  }

  public double getShooterRPM()
  {
    return shooting_motor.getSelectedSensorVelocity();
  }

  // FF & PID for shooter motor to set RPM
  // https://trickingrockstothink.com/blog_posts/2019/10/19/tuning_pid.html

  /** Returns true if a power cell is being shot */
  public boolean powerCellFired()
  {
    return !shooter_sensor_top.get();
  }

  // TODO Maybe have a way to report the amount of balls in storage depending on sensor layout

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Eject RPM", getShooterRPM());

    // Run ejector if we're asked to do it,
    // or for 2 more seconds after the last shot
    // so it remains running through a series of shots
    if (shoot || keep_running_timer.get() < 2.0)
      setShooterVoltage(SHOOTER_VOLTAGE);
    else
      setShooterVoltage(0);
  }
}
