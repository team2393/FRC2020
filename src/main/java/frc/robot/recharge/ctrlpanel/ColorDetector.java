/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

/** Basic color detector interface */
public interface ColorDetector
{
  /** Color names for color index 0, 1, 2, 3 */
  public static final String[] COLORS = { "Blue", "Green", "Red", "Yellow" };
  
  /** @return 0, 1, 2, 3 for color, -1 if we don't know */
  int getColor();
}
