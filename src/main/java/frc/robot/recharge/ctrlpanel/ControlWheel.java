/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.RobotMap;
import frc.robot.recharge.test.ControlWheelTestRobot;

/** Control panel wheel */
public class ControlWheel extends SubsystemBase implements ColorDetector
{
  private final WPI_TalonFX motor = new WPI_TalonFX(RobotMap.CONTROL_PANEL_WHEEL);
  private final Solenoid extender = new Solenoid(RobotMap.CONTROL_PANEL_SOLENOID);

  private final ColorDetector detector = new ColorSensor();

  public ControlWheel()
  {
    motor.configFactoryDefault();
  }

  /** @param extent Extend the control wheel assembly? */
  public void extend(final boolean extend)
  {
    extender.set(extend);
  }

  /** Turn the wheel
   *  @param speed Speed -1 .. 1
   */
  public void spin(final double speed)
  {
    motor.set(speed);
    motor.setInverted(true);
  }

  public void slow()
  {
    spin(0.01);
  }

  public void fast()
  {
    spin(0.1);
  }

  public Segment_Color getColor()
  {
    return detector.getColor();
  }

  @Override
  public void periodic()
  {
    System.out.println("Color: " + getColor());
  }
}