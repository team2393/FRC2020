/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.function.Function;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Build auto commands from a file */
public class AutonomousBuilder
{
  /** @param filename File to read
   *  @param trajectory_command Function that turns trajectory into command
   *  @throws Exception on error
   */
  public static List<SequentialCommandGroup> read(final File filename,
                                                  final Function<Trajectory, CommandBase> trajectory_command) throws Exception
  {
    final BufferedReader file = new BufferedReader(new FileReader(filename));
    final List<SequentialCommandGroup> autos = new ArrayList<>();
    SequentialCommandGroup current_auto = null;

    // TODO Don't reset drivetrain.
    // If we reset position for each trajectory, the small errors at end of trajectory follower add up.
    // Get current position,
    // transform each trajectory relative to the position at the time,
    // and move 'current' position to the desired end point of each trajectory.
    String line;
    while ((line = file.readLine()) != null)
    {
      // Skip empty lines and comments
      if (line.isBlank()  ||  line.startsWith("#"))
        continue;
  
      Scanner scanner = new Scanner(line);
      final String command = scanner.next();
      if (command.startsWith("Auto"))
      { // Auto Name-of-this-sequence:
        // Start new auto
        current_auto = new SequentialCommandGroup();
        current_auto.setName(scanner.nextLine());
        autos.add(current_auto);
        System.out.println("Reading Auto '" + current_auto.getName() + "''");
      }
      else if (command.startsWith("Traj") ||
               command.startsWith("ReverseTr"))
      { // Trajectory:
        // Read trajectory info
        final Trajectory trajectory = TrajectoryReader.read(file, command.startsWith("Reverse"));
        // Turn into command, which may be a 'print' or a 'ramsete' that follows it
        current_auto.addCommands(trajectory_command.apply(trajectory));
        System.out.println("Added Trajectory");
      }
      else if (command.startsWith("PathW")  ||
               command.startsWith("ReverseW"))
      { // Read trajectory from PathWeaver file
        final Path pwfile = new File(Filesystem.getDeployDirectory(), scanner.next()).toPath();
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(pwfile);
        // We need traj. starting at X 0, Y 0, Heading 0, so move relative to start point
        trajectory = trajectory.relativeTo(trajectory.sample(0).poseMeters);

        if (command.startsWith("ReverseW"))
          trajectory = TrajectoryHelper.reverse(trajectory);

        current_auto.addCommands(trajectory_command.apply(trajectory));
        System.out.println("Added " + command + " " + pwfile);
      }
      else
        throw new Exception("Unknown autonomouse command: " + line);

      scanner.close();
    }

    return autos;
  }
  
  public static void main(String[] args) throws Exception
  {
    // Open demo file
    final List<SequentialCommandGroup> autos =
     AutonomousBuilder.read(new File("src/main/deploy/auto.txt"),
                            trajectory -> new PrintCommand("Trajectory to " + TrajectoryHelper.getEndInfo(trajectory)));

    for (SequentialCommandGroup auto : autos)
    {
      System.out.println("*** Auto: " + auto.getName());
      auto.initialize();
      // As long as demo only contains PrintCommand, we can execute it on the laptop!
      auto.execute();
    }
  }
}