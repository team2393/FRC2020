/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
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
  // TODO figure out what type of motor controllers will actually be used
  private final WPI_TalonFX shooting_motor = new WPI_TalonFX(RobotMap.SHOOTER_MOTOR);
  private final WPI_TalonSRX conveyor_motor = new WPI_TalonSRX(RobotMap.CONVEYOR_MOTOR);
  private final WPI_TalonSRX intake_motor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);
  private final WPI_TalonSRX angle_adjustment = new WPI_TalonSRX(RobotMap.ANGLE_MOTOR);
  
  // Sensors: Banner T8??6D50
  private final DigitalInput ball_at_end_of_conveyor = new DigitalInput(RobotMap.BALL_AT_END_OF_CONVEYOR);
  private final DigitalInput ball_at_end_of_ejector = new DigitalInput(RobotMap.BALL_AT_END_OF_EJECTOR);
  
  public void setShooterSpeed(double speed) 
  {
    shooting_motor.set(speed);  
  }

  public void moveConveyor(double speed)
  {
    // Should the conveyor have the ability to move backwards or should it only be "on" or "off"
    conveyor_motor.set(speed);
  }
  
  public void setIntakeSpeed(double speed)
  {
    intake_motor.set(Math.abs(speed)); // TODO find out which way motor spins and only allow intake to spin one direction
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

  /** @return true if there's a ball at the end of the (vertical) conveyer belt,
   *          just before the shooter.
   *          If converyor keeps moving, that ball will be ejected
   */
  public boolean isStorageFull()
  {
    // Sensor is inverted, 'false' == seeing a ball
    return !ball_at_end_of_conveyor.get();
  }
  
  /** @return true if a ball is detected at the ejector
   *          (will only be briefly true since ball is then ejected)
   */
  public boolean isBallEjected()
  {
    // Sensor is inverted, 'false' == seeing a ball
    return !ball_at_end_of_ejector.get();
  }

  // TODO Maybe have a way to report the amount of balls in storage depending on sensor layout
}
