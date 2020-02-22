/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Conveyor and ejector
 * 
 *  'Fuel cell' balls drop from hopper onto horizontal conveyor belt.
 *  Horizontal belt moves them to vertical conveyor,
 *  which feeds them to ejector/shooter.
 * 
 *  Maintains a spinner for shooting balls,
 *  keeping it running for a little longer after last shot
 *  in case we then soon need it again.
 */
public class PowerCellAccelerator extends SubsystemBase 
{
  // Motors
  private final Spinner shooter = new Spinner();
  private final WPI_VictorSPX conveyor_top = new WPI_VictorSPX(RobotMap.CONVEYOR_TOP);
  private final WPI_VictorSPX conveyor_bottom = new WPI_VictorSPX(RobotMap.CONVEYOR_BOTTOM); 
  
  // Sensors
  private final DigitalInput shooter_sensor_ready = new DigitalInput(RobotMap.SHOOTER_SENSOR_READY);
  private final DigitalInput shooter_sensor_eject = new DigitalInput(RobotMap.SHOOTER_SENSOR_EJECT);

  /** Normal voltage for moving conveyors */
  public final static double CONVEYOR_VOLTAGE = 11.0;

  /** Ejector spinner setpoint */
  public final static double SHOOTER_RPM = 5000;

  /** Minimum speed for shooting a ball as fraction of SHOOTER_RPM */
  public final static double MINIMUM_RPM_FRACTION = 0.9;

  /** Should we shoot? */
  private boolean shoot = false;
  /** Timer started when 'shoot' clears to keep the ejector running */
  private final Timer keep_running_timer = new Timer();
  /** Is the timer running? */
  private boolean timer_on = false;

  public PowerCellAccelerator()
  {
    commonSettings(conveyor_top, NeutralMode.Brake);
    commonSettings(conveyor_bottom, NeutralMode.Brake);
  }
  
  /** @param motor Motor to configure with common settings
   *  @param mode Neutral mode
   */
  static void commonSettings(final BaseMotorController motor, final NeutralMode mode)
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(mode);
    motor.setInverted(true);
  }
  
  public void moveConveyor(final double volt)
  {
    // Conveyors should probably be moved seperately
    conveyor_bottom.setVoltage(volt);
    conveyor_top.setVoltage(volt);
  }
  
  /** Returns true if power cell is in "ready" position at end of vertical conveyor */
  public boolean powerCellReady()
  {
    return !shooter_sensor_ready.get();
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
    {
      keep_running_timer.start();
      timer_on = true;
    }
        
    shoot = on_off;
  }

  public double getShooterRPM()
  {
    return shooter.getRPM();
  }

  /** Returns true if a power cell is being shot */
  public boolean powerCellFired()
  {
    return !shooter_sensor_eject.get();
  }

  @Override
  public void periodic()
  {
     // Run ejector if we're asked to do it,
    // or for 2 more seconds after the last shot
    // so it remains running through a series of shots
    if (shoot || (timer_on && keep_running_timer.get() < 2.0))
      shooter.setRPM(SHOOTER_RPM);
    else
    {
      shooter.setVoltage(0);
      timer_on = false;
    }
  }
}
