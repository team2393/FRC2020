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
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.recharge.drivetrain.DriveTrain;

/** Tool for reading trajectory info from a file */
public class TrajectoryReader
{
  // TODO Try various curvature constraints.
  //      Also check speed PID (P) and wheel base track width from characterization
  public static TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0)
                                                                      .addConstraint(new CurvatureConstraint(90.0))
                                                                      // .setKinematics(DriveTrain.kinematics)
                                                                      ;

  /** Read a trajectory from a file with "Point X Y" and "End X Y Heading" commands
   * 
   *  Will read trajectory info from the current position of the file reader until
   *  reaching an end-of-trajactory line.
   * 
   *  @param file File reader
   *  @param reverse Going backwards along the trajectory?
   *  @return {@link Trajectory}
   *  @throws Exception on error
   */
  public static Trajectory readPoints(final BufferedReader file, final boolean reverse) throws Exception
  {
    // Assume we're moving forward
    config.setReversed(reverse);
    // Initial position and heading
    final Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Intermediate points (no heading)
    final List<Translation2d> waypoints = new ArrayList<>();
    
    // Track 'current' position (may be used to add waypoints)
    Translation2d pos = start.getTranslation();

    // End position, must be set when we're "done"
    Pose2d end = null;
    
    String line;
    while (end == null  &&  (line = file.readLine()) != null)
    {
      // Skip empty lines and comments
      if (line.isBlank()  ||  line.startsWith("#"))
        continue;
  
      Scanner scanner = new Scanner(line);
      final String command = scanner.next();
      if (command.equals("Point"))
      { // Point X Y (absolute)
        pos = new Translation2d(scanner.nextDouble(),
                                scanner.nextDouble());
        waypoints.add(pos);                     
      }
      else if (command.equals("RPoint"))
      { // RelativePoint X Y
        pos = pos.plus(new Translation2d(scanner.nextDouble(),
                                         scanner.nextDouble()));
        waypoints.add(pos);                     
      }
      else if (command.equals("End"))
      { // End X Y Heading
        end = new Pose2d(scanner.nextDouble(),
                         scanner.nextDouble(),
                         Rotation2d.fromDegrees(scanner.nextDouble()));
      }
      else
        throw new Exception("Unknown trajectory command:" + line);
      scanner.close();
    }

    if (end == null)
      throw new Exception("Trajectory lacks 'End X Y Heading'");
    
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
  }

  /** Read a trajectory from a file with "Pose X Y Heading" commands
   * 
   *  Will read trajectory info from the current position of the file reader until
   *  reaching an end-of-trajactory line.
   * 
   *  @param file File reader
   *  @param reverse Going backwards along the trajectory?
   *  @return {@link Trajectory}
   *  @throws Exception on error
   */
  public static Trajectory readPoses(final BufferedReader file, final boolean reverse) throws Exception
  {
    // Assume we're moving forward
    config.setReversed(reverse);

    final List<Pose2d> points = new ArrayList<>();
    points.add(new Pose2d());
    
    // End position, must be set when we're "done"
    Pose2d end = null;
    
    String line;
    while (end == null  &&  (line = file.readLine()) != null)
    {
      // Skip empty lines and comments
      if (line.isBlank()  ||  line.startsWith("#"))
        continue;
  
      Scanner scanner = new Scanner(line);
      final String command = scanner.next();
      if (command.equals("Pose")  ||  command.equals("End"))
      { // Point X Y (absolute)
        final Pose2d pose = new Pose2d(scanner.nextDouble(),
                                       scanner.nextDouble(),
                                       Rotation2d.fromDegrees(scanner.nextDouble()));
        points.add(pose);
        if (command.equals("End"))
          end = pose;
      }
      else
        throw new Exception("Unknown trajectory command:" + line);
      scanner.close();
    }

    if (end == null)
      throw new Exception("Trajectory lacks 'End X Y Heading'");
    
    return TrajectoryGenerator.generateTrajectory(points, config);
  }
  
  /** Read poses from a PathWeaver *.path file,
   *  then create trajectory.
   * 
   *  The result may be a little off the PathWeaver-generated
   *  trajectory, but can take advantage of 'constraints'.
   * 
   *  @param file Well, the file to read
   *  @return {@link Trajectory}
   *  @throws Exception on error
   */
  public static Trajectory readPath(final File file) throws Exception
  {
    final List<Pose2d> points = new ArrayList<>();
    try ( final Scanner scanner = new Scanner(file) )
    {
      scanner.useDelimiter("\\s*,\\s*");
      // Skip the first "X,Y,Tangent X,Tangent Y,Fixed Theta,Name" line
      scanner.nextLine();
      while (scanner.hasNextLine())
      {
        final double x = scanner.nextDouble();
        final double y = scanner.nextDouble();
        final double dx = scanner.nextDouble();
        final double dy = scanner.nextDouble();
        points.add(new Pose2d(x, y, new Rotation2d(dx, dy)));
        scanner.nextLine();
      }
    }
    return TrajectoryGenerator.generateTrajectory(points, config);
  }

  public static void main(String[] args) throws Exception
  {
    Trajectory trajectory = readPath(new File("src/main/deploy/PathWeaver/Paths/Test.path"));
    trajectory = TrajectoryHelper.makeTrajectoryStartAt(trajectory, new Pose2d());
    for (State state : trajectory.getStates())
      System.out.println(state);
    new TrajectoryViewer(trajectory, 0.1);

    // Open demo file
    final BufferedReader file = new BufferedReader(new FileReader("src/main/deploy/demo_trajectory.txt"));
    trajectory = TrajectoryReader.readPoses(file, false);
    double min = 0, max = 0;
    for (State state : trajectory.getStates())
    {
      // System.out.println(state);
      final double degree_per_second = Math.toDegrees(state.velocityMetersPerSecond * state.curvatureRadPerMeter);
      min = Math.min(min, degree_per_second);
      max = Math.max(max, degree_per_second);
    }
    System.out.format("Rotation: %.2f .. %.2f degrees/sec\n", min, max);

    // Show it
    new TrajectoryViewer(trajectory, 0.1);
  }
}