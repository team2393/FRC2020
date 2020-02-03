/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.auto;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ApplySettings extends InstantCommand
{
  private final Map<String, Double> settings = new HashMap<>();
  private final File file;

  public ApplySettings(final String filename)
  {
    file = new File(Filesystem.getDeployDirectory(), filename);
    try
    {
      // Read the settings from file
      Scanner scanner = new Scanner(file);
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
      
      scanner.close();
    }
    catch (Exception ex)
    {
      System.out.println("Cannot read " + file);
      ex.printStackTrace();
    }
  }
  
  @Override
  public void execute()
  {
    // Apply the settings!
    for (String setting : settings.keySet())
      {
        Double value = settings.get(setting);
        SmartDashboard.putNumber(setting, value);
        System.out.println("Setting " + setting + " to " + value);
      }
  }
}
