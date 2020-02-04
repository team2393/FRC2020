/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.udp;

/**
 * Add your docs here.
 */
public class CameraData 
{
  public int direction;
  public int distance;
  
  public CameraData(int direction, int distance)
  {
    this.direction = direction;
    this.distance = distance;
  }

  @Override
  public String toString() 
  {
    return ("Direction: " + direction + " Distance: " + distance);
  }
}
