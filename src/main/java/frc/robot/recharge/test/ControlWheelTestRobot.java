/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ExtendControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RetractControlWheel;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;

/** Robot code for testing control wheel */
public class ControlWheelTestRobot extends BasicRobot
{
  private final ControlWheel wheel = new ControlWheel();
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    OI.extend_control_wheel.whenPressed(new ExtendControlWheel(wheel));
    OI.retract_control_wheel.whenPressed(new RetractControlWheel(wheel));
    OI.enable_wheel.whenPressed(new ManualWheelSpeed(wheel));
    OI.autorotate_wheel.whenPressed(new RotateWheel(wheel, 3));
    OI.rotate_to_color.whenPressed(new RotateToColor(wheel));
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putString("Color", wheel.getColor().name());
  }
}
