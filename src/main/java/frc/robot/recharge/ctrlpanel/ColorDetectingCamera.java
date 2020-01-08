/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorDetectingCamera implements ColorDetector
{
  /** Last pipeline calls that we saw */
  private int last_calls = -1;

  /** Read color sensor
   *  @return Detector color index or -1
   */
  @Override
  public int getColor()
  {
        // Do we have new image information?
        int calls = (int) SmartDashboard.getNumber("PipelineCalls", -1);
        if (calls == last_calls)
        {
          // System.out.println("Stale image info");
          return -1;
        }
        last_calls = calls;
    
        // Did the camera detect a color?
        return (int) SmartDashboard.getNumber("Color Idx", -1);
  }
}