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

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/** Tool for reading trajectory info from a file */
public class TrajectoryReader
{
  public static TrajectoryConfig config = new TrajectoryConfig(1.0, 0.3);

  /** Read a trajectory from a file.
   * 
   *  Will read trajectory info from the current position of the file reader until
   *  reaching an end-of-trajactory line.
   * 
   *  @param file File reader
   *  @return {@link Trajectory}
   *  @throws Exception on error
   */
  public static Trajectory read(final BufferedReader file) throws Exception
  {
    // Initial position and heading
    final Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Intermediate points (no heading)
    final List<Translation2d> waypoints = new ArrayList<>();
    
    // Track 'current' position (may be used to add waypoints)
    Translation2d pos = start.getTranslation();

    // End position, must be set when we're "done"
    Pose2d end = null;
    
    String line;
    while ((line = file.readLine()) != null  &&  end == null)
    {
      // Skip empty lines and comments
      if (line.isBlank()  ||  line.startsWith("#"))
        continue;
  
      Scanner scanner = new Scanner(line);
      final String command = scanner.next();
      if (command.startsWith("P"))
      { // Point X Y (absolute)
        pos = new Translation2d(scanner.nextDouble(),
                                scanner.nextDouble());
        waypoints.add(pos);                     
      }
      else if (command.startsWith("RP"))
      { // RelativePoint X Y
        pos = pos.plus(new Translation2d(scanner.nextDouble(),
                                         scanner.nextDouble()));
        waypoints.add(pos);                     
      }
      else if (command.startsWith("E"))
      { // End X Y Heading
        end = new Pose2d(scanner.nextDouble(),
                         scanner.nextDouble(),
                         Rotation2d.fromDegrees(scanner.nextDouble()));
      }
      scanner.close();
    }

    if (end == null)
      throw new Exception("Trajectory lacks 'End X Y Heading'");
    
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
  }
  
  public static void main(String[] args) throws Exception
  {
    // Open demo file
    final BufferedReader file = new BufferedReader(new FileReader("src/main/deploy/demo_trajectory.txt"));
    final Trajectory trajectory = TrajectoryReader.read(file);

    // Show it
    new TrajectoryViewer(trajectory, 0.1);
  }
}