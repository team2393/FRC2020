/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/** Helpers for dealing with Trajectory */
public class TrajectoryHelper
{
  /** @param trajectory Trajectory
   *  @return End state
   */
  public static State getEnd(final Trajectory trajectory)
  {
    final List<State> states = trajectory.getStates();
    return states.get(states.size()-1);
  }

  /** @param state One state along a Trajectory
   *  @return "X=..., Y=..., ..."
   */
  public static String getInfo(final State state)
  {
    return String.format("X=%.2f m, Y=%.2f m, Heading=%.1f degrees",
                         state.poseMeters.getTranslation().getX(),
                         state.poseMeters.getTranslation().getY(),
                         state.poseMeters.getRotation().getDegrees());
  }

  /** @param trajectory Trajectory;
   *  @return Info about end point
   */
  public static String getEndInfo(final Trajectory trajectory)
  {
    return "End: " + getInfo(getEnd(trajectory));
  }

  /** Revert a trajectory
   * 
   *  Change end point to start point,
   *  next-to-last into second point.
   *  Inverts direction of velocities etc,
   *  keeping X/Y coordinates.
   * 
   *  Meant to turn existing 'forward' trajectory
   *  into one that's used in 'reverse', going
   *  backwards.
   * 
   * @param original Trajectory
   * @return Reversed trajectory
   */
  public static Trajectory reverse(final Trajectory original)
  {
    final List<State> orig_states = original.getStates();
    final List<State> reversed_states = new ArrayList<>();
    final double duration = original.getTotalTimeSeconds();

    // Run through original states in reverse order
    for (int i=orig_states.size()-1;  i>=0;  --i)
    {
      final State orig = orig_states.get(i);
      reversed_states.add(new State(duration - orig.timeSeconds,
                                    -orig.velocityMetersPerSecond,
                                    -orig.accelerationMetersPerSecondSq,
                                     orig.poseMeters,
                                    -orig.curvatureRadPerMeter));
    }
    return new Trajectory(reversed_states);
  }
}