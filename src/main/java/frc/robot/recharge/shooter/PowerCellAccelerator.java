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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.OI;
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
  enum Mode
  {
    Off,
    Arm,
    Shoot
  };

  private Mode mode = Mode.Off;

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

  // private static final double TICKS_PER_DEGREE = 0 / Units.inchesToMeters(0);

  public void PowerCellAccelerator()
  {
    commonSettings(shooting_motor);
    commonSettings(conveyor_top);
    commonSettings(conveyor_bottom);
    commonSettings(intake_position);
    commonSettings(angle_adjustment);
  }

  /** @param motor Motor to configure with common settings */
  private void commonSettings(final WPI_TalonFX motor)
  {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralMode.Coast);
  }

  public void setShooterSpeed(double speed) 
  {
    shooting_motor.set(speed);  
  }

  public void moveConveyor(double speed)
  {
    // Should the conveyor have the ability to move backwards or should it only be "on" or "off"
    // Conveyors should probably be moved seperately
    conveyor_bottom.set(speed);
    conveyor_top.set(speed);
  }
  
  public void setIntakeSpeed(double speed)
  {
    intake_motor.set(-Math.abs(speed)); // TODO find out which way motor spins and only allow intake to spin one direction
  }
  
  public void setAngle(int angle)
  {
    // Calculate angle with encoder values and use PID to adjust
  }

  public double getShooterVelocity()
  {
    // TODO command that waits for shooter motor to reach a certain velocity before loading ball
    return shooting_motor.getSelectedSensorVelocity();
  }

  // TODO Maybe turn this into a int that returns amount of balls in storage depending on sensor layout
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
    if (mode == Mode.Off)
    {
      // moveConveyor(0.0);
    }
    else if (mode == Mode.Arm)
    {
      // if (.. sensor ..)
      //   ...
    }
  }
}
