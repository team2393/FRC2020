/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.auto;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Apply settings from file to network tables */
public class ApplySettings extends InstantCommand
{
  private final Map<String, Double> settings = new HashMap<>();
  private final File file;

  public ApplySettings(final String filename)
  {
    file = new File(Filesystem.getDeployDirectory(), filename);
    System.out.println("Settings file: " + filename);
    try
    (
      Scanner scanner = new Scanner(file);
    )
    {
      // Read the settings from file
      scanner.useDelimiter("[ \t\r\n]+");
      while (scanner.hasNextLine())
      {
        String setting = scanner.next();
        // Ignore comments
        if (setting.startsWith("#"))
          scanner.nextLine();
        else
        {
          double value = scanner.nextDouble();
          // Remember for later when we execute()
          settings.put(setting, value);
          System.out.println("Reading " + setting + " = " + value);
        }
      }
    }
    catch (Exception ex)
    {
      System.out.println("Cannot read " + file);
      ex.printStackTrace();
    }
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  @Override
  public void execute()
  {
    // Apply the settings!
    System.out.println("Applying settings:");
    for (String setting : settings.keySet())
    {
      double value = settings.get(setting);
      SmartDashboard.putNumber(setting, value);
      System.out.println("Setting " + setting + " to " + value);
    }
  }
}
