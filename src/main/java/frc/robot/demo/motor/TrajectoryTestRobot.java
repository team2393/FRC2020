/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.motor;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.BasicRobot;

/** Example of PID control for position
 * 
 *  Uses the Falcon's encoder
 *  with PID control performed in the RoboRIO
 */
public class TrajectoryTestRobot extends BasicRobot
{
    
  @Override
  public void robotInit()
  {
    super.robotInit();

    // Describe where we want to go
    //
    //  Y
    // /|\           End
    //  |
    //  |            P2
    //  |
    //  | P1
    //  |
    // Start ----------> X
    // Units are meters.
    TrajectoryConfig config = new TrajectoryConfig(1.0, 1.0);

    // Could use Units.feetToMeters() to convert
    final Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
    Translation2d pos = start.getTranslation();
    
    final List<Translation2d> waypoints = new ArrayList<>();
    pos = pos.plus(new Translation2d(0, 5));
    // waypoints.add(pos);
    
    pos = pos.plus(new Translation2d(5, 5));
    // waypoints.add(pos);
    
    pos = pos.plus(new Translation2d(0, 5));
    final Pose2d end = new Pose2d(pos, Rotation2d.fromDegrees(90.0));
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    // final List<Pose2d> waypoints = new ArrayList<>();
    // final Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
    // waypoints.add(start);


    // final Pose2d end = new Pose2d(15.0, 5.0, Rotation2d.fromDegrees(90.0));
    // waypoints.add(end);

    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    
    final double total_time = trajectory.getTotalTimeSeconds();
    System.out.println("************* Trajectory ******************");
    System.out.format("Trajectory time: %.1f sec\n", total_time);
    System.out.println("Time  X  Y  Heading");
    for (double time=0; time < total_time+1;  time += 1)
      show(trajectory.sample(time));
    System.out.println("*******************************************");
  }

  private void show(final State state)
  {
    System.out.format("%5.1f %.1f %.1f %.1f\n",
                      state.timeSeconds,
                      state.poseMeters.getTranslation().getX(),
                      state.poseMeters.getTranslation().getY(),
                      state.poseMeters.getRotation().getDegrees()
                      );
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
  }
  
  @Override
  public void autonomousPeriodic()
  {
  }
}