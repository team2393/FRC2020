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
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // Must have encoder (speed)
  private final WPI_TalonFX shooting_motor = new WPI_TalonFX(RobotMap.SHOOTER_MOTOR);
  private final WPI_TalonFX conveyor_top = new WPI_TalonFX(RobotMap.CONVEYOR_TOP);
  private final WPI_TalonFX conveyor_bottom = new WPI_TalonFX(RobotMap.CONVEYOR_BOTTOM); 
  private final WPI_VictorSPX intake_motor = new WPI_VictorSPX(RobotMap.INTAKE_MOTOR);
  // Must have encoder (angle) and limit switch (home)
  private final WPI_TalonFX intake_position = new WPI_TalonFX(RobotMap.INTAKE_POSITION);
  // Must have encoder (angle)
  private final WPI_TalonFX angle_adjustment = new WPI_TalonFX(RobotMap.ANGLE_MOTOR);
  
  // Sensors
  private final DigitalInput shooter_sensor_mid = new DigitalInput(RobotMap.SHOOTER_SENSOR_MID);
  private final DigitalInput shooter_sensor_top = new DigitalInput(RobotMap.SHOOTER_SENSOR_TOP);

  // PID
  // TODO Tune, then turn into ProfiledPIDController
  private final PIDController intake_position_pid = new PIDController(0, 0, 0);
  private final PIDController shooter_angle_pid = new PIDController(0, 0, 0);

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
    commonSettings(intake_motor, NeutralMode.Coast);
    commonSettings(intake_position, NeutralMode.Brake);
    commonSettings(angle_adjustment, NeutralMode.Brake);

    // Encoder for speed resp. position
    shooting_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    intake_position.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);    
    angle_adjustment.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Limit switch to 'home'
    intake_position.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }
  
  /** @param motor Motor to configure with common settings
   *  @param mode Neutral mode
   */
  private void commonSettings(final BaseMotorController motor, final NeutralMode mode)
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(mode);
  }
  
  /** Move intake up towards 'home' switch.
   *  @return true if at 'home' position
   */
  public boolean homeIntake()
  {
    // TODO Find voltage for slow movement towards home switch
    // TODO Is home switch the forward or reverse limit switch?
    intake_position.setVoltage(-0.1);
    final boolean homed = intake_position.isFwdLimitSwitchClosed() == 1;
    if (homed)
     intake_position.setSelectedSensorPosition(0);
    return homed;
  }

  private double getIntakeAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle
    return intake_position.getSelectedSensorPosition();
  }

  public void setIntakeAngle(final double angle)
  {
    // TODO First connect joystick axis to 'angle' to simply move motor
    intake_position.setVoltage(angle);

    // Calibrate angle
    // Then change this to
    //    intake_position_pid.setSetpoint(angle);
    // enable PID in periodic() below and tune PID.
    // Then change to ProfiledPIDController
  }
  
  // TODO Command to lower intake and turn rollers on,
  //      or raise intake with rollers off.
  public void setIntakeSpeed(final double volt)
  {
    intake_motor.setVoltage(-Math.abs(volt)); // TODO find out which way motor spins and only allow intake to spin one direction
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

  private double getHoodAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle
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

  /** Returns true if a power cell is being shot */
  public boolean powerCellFired()
  {
    return !shooter_sensor_top.get();
  }

  // TODO Maybe have a way to report the amount of balls in storage depending on sensor layout

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    SmartDashboard.putNumber("Eject RPM", getShooterRPM());

    // TODO See setIntakeAngle(angle);
    // intake_motor.setVoltage(intake_position_pid.calculate(getIntakeAngle()));

    // TODO See setHoodAngle(angle);
    // angle_adjustment.setVoltage(shooter_angle_pid.calculate(getHoodAngle()));

    // Run ejector if we're asked to do it,
    // or for 2 more seconds after the last shot
    // so it remains running through a series of shots
    if (shoot || keep_running_timer.get() < 2.0)
      setShooterVoltage(SHOOTER_VOLTAGE);
    else
      setShooterVoltage(0);
  }
}
