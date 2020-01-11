/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.RobotMap;

/** Control panel wheel */
public class ControlWheel extends SubsystemBase implements ColorDetector
{
  private final Servo motor = new Servo(RobotMap.CONTROL_PANEL_WHEEL);

  final ColorDetector 
  detector = new ColorSensor();
                                 // new ColorDetectingCamera();

  /** Turn the wheel
   *  @param speed Speed -1 .. 1
   */
  public void spin(final double speed)
  {
    // Convert -1..1 range into 0..1 range
    motor.set((MathUtil.clamp(speed, -1.0, 1.0) + 1.0) / 2.0);
  }

  public void slow()
  {
    spin(-0.01);
  }

  public void fast()
  {
    spin(-0.1);
  }

  public int getColor()
  {
    return detector.getColor();
  }
}