/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

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

  /** @param trajectory Trajectory
   *  @return Info about end point
   */
  public static String getEndInfo(final Trajectory trajectory)
  {
    return "End: " + getInfo(getEnd(trajectory));
  }
}