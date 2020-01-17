/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Build auto commands from a file */
public class AutonomousBuilder
{
  /** @param filename File to read
   *  @throws Exception on error
   */
  public static List<SequentialCommandGroup> read(final String filename) throws Exception
  {
    final BufferedReader file = new BufferedReader(new FileReader(filename));
    final List<SequentialCommandGroup> autos = new ArrayList<>();
    SequentialCommandGroup current_auto = null;

    String line;
    while ((line = file.readLine()) != null)
    {
      // Skip empty lines and comments
      if (line.isBlank()  ||  line.startsWith("#"))
        continue;
  
      Scanner scanner = new Scanner(line);
      final String command = scanner.next();
      if (command.startsWith("A"))
      { // Auto Name-of-this-sequence:
        // Start new auto
        current_auto = new SequentialCommandGroup();
        current_auto.setName(scanner.nextLine());
        autos.add(current_auto);
      }
      else if (command.startsWith("T"))
      { // Trajectory:
        // Read trajectory info, add command that follows it
        final Trajectory trajectory = TrajectoryReader.read(file);
        final String info = "Trajectory to " + TrajectoryHelper.getEndInfo(trajectory);
        // TODO: Should add RamseteCommand(trajectory, ..., )
        current_auto.addCommands(new PrintCommand(info));
      }
      scanner.close();
    }

    return autos;
  }
  
  public static void main(String[] args) throws Exception
  {
    // Open demo file
    final List<SequentialCommandGroup> autos = AutonomousBuilder.read("src/main/deploy/auto.txt");
    for (SequentialCommandGroup auto : autos)
    {
      System.out.println("*** Auto: " + auto.getName());
      auto.initialize();
      // TODO As long as demo only contains PrintCommand, we can execute it!
      auto.execute();
    }
  }
}