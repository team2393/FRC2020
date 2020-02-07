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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling
 * 
 *  Intake picks 'fuel cell' balls from floor into hopper.
 *  From hopper they drop onto horizontal conveyor belt.
 *  Horizontal belt moves them to vertical conveyor,
 *  which feeds them to ejector/shooter.
 *  Angle of rotatable hood/shield/deflector adjusts
 *  the angle at which balls are ejected.
 */
public class PowerCellAccelerator extends SubsystemBase 
{
  // Motors
  // TODO figure out what type of motor controllers will actually be used -- Tony was leaning towards falcons for most
  private final WPI_TalonFX shooting_motor = new WPI_TalonFX(RobotMap.SHOOTER_MOTOR);
  private final WPI_TalonFX conveyor_top = new WPI_TalonFX(RobotMap.CONVEYOR_TOP);
  private final WPI_TalonFX conveyor_bottom = new WPI_TalonFX(RobotMap.CONVEYOR_BOTTOM); 
  private final WPI_VictorSPX intake_motor = new WPI_VictorSPX(RobotMap.INTAKE_MOTOR);
  private final WPI_TalonFX intake_position = new WPI_TalonFX(RobotMap.INTAKE_POSITION);
  private final WPI_TalonFX angle_adjustment = new WPI_TalonFX(RobotMap.ANGLE_MOTOR);
  
  // Sensors
  private final DigitalInput shooter_sensor_mid = new DigitalInput(RobotMap.SHOOTER_SENSOR_MID);
  private final DigitalInput shooter_sensor_top = new DigitalInput(RobotMap.SHOOTER_SENSOR_TOP);

  // PID
  private final PIDController intake_position_pid = new PIDController(0, 0, 0);
  private final PIDController shooter_angle_pid = new PIDController(0, 0, 0);

  public final static double CONVEYOR_VOLTAGE = 5.0;

  public final static double SHOOTER_VOLTAGE = 10.5;

  public final static double MINIMUM_SHOOTER_RPM = 5000;

  private final Timer shoot_timer = new Timer();
  private boolean shoot = false;

  public PowerCellAccelerator()
  {
    commonSettings(shooting_motor, NeutralMode.Brake);
    commonSettings(conveyor_top, NeutralMode.Coast);
    commonSettings(conveyor_bottom, NeutralMode.Coast);
    commonSettings(intake_position, NeutralMode.Coast);
    commonSettings(angle_adjustment, NeutralMode.Coast);

    shooting_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    intake_position.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    angle_adjustment.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  /** @param motor Motor to configure with common settings */
  private void commonSettings(final WPI_TalonFX motor, final NeutralMode mode)
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(mode);
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
    shoot = on_off;
    if (shoot)
      shoot_timer.start();
  }

  public void setShooterVoltate(double volt) 
  {
    shooting_motor.setVoltage(volt);  
  }

  public void moveConveyor(double volt)
  {
    // Should the conveyor have the ability to move backwards or should it only be "on" or "off"
    // Conveyors should probably be moved seperately
    conveyor_bottom.setVoltage(volt);
    conveyor_top.setVoltage(volt);
  }
  
  public void setIntakeSpeed(double volt)
  {
    intake_motor.setVoltage(-Math.abs(volt)); // TODO find out which way motor spins and only allow intake to spin one direction
  }
  
  public void setAngle(int angle)
  {
    // Calculate angle with encoder values and use PID to adjust
  }

  public double getShooterRPM()
  {
    return shooting_motor.getSelectedSensorVelocity();
  }

  /** Returns true if a power cell is being shot */
  public boolean powerCellFired()
  {
    return !shooter_sensor_top.get();
  }

  /** Returns true if power cell is in "ready" position */
  public boolean powerCellReady()
  {
    return !shooter_sensor_mid.get();
  }

  /** @return true if there's a ball at the end of the (vertical) conveyer belt,
   *          just before the shooter.
   *          If converyor keeps moving, that ball will be ejected
  public boolean isStorageFull()
  {
    // Sensor is inverted, 'false' == seeing a ball
    return !shooter_sensor_top.get();
  }
   */
  
  /** @return true if a ball is detected at the ejector
   *          (will only be briefly true since ball is then ejected)
  public boolean isBallEjected()
  {
    // Sensor is inverted, 'false' == seeing a ball
    return !ball_at_end_of_ejector.get();
  }
   */

  // TODO Maybe have a way to report the amount of balls in storage depending on sensor layout

  @Override
  public void periodic()
  {
    // Run ejector if we're asked to do it,
    // or for 2 more seconds after the last shot
    // so it remains running through a series of shots
    if (shoot || shoot_timer.get() < 2.0)
      setShooterVoltate(SHOOTER_VOLTAGE);
    else
      setShooterVoltate(0);
  }
}
