/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/** Plot trajectory on laptop
 * 
 *  Can be used by TrajectoryReader
 */
public class TrajectoryViewer
{
  private final Trajectory trajectory;
  private final double time_step;

  private class TrajectoryPlot extends JPanel
  {
    @Override
    protected void paintComponent(final Graphics g)
    {
      // Determine bounding box of trajectory
      double xmin = 0, xmax = 1, ymin=0, ymax = 1, speedmax = 0;
      for (State state : trajectory.getStates())
      {
        final double x = state.poseMeters.getTranslation().getX(),
                     y = state.poseMeters.getTranslation().getY();
        xmin = Math.min(xmin, x);
        xmax = Math.max(xmax, x);
        ymin = Math.min(ymin, y);
        ymax = Math.max(ymax, y);
        speedmax = Math.max(speedmax, state.velocityMetersPerSecond);
      }
      // Largest size (width or height) of trajectory
      final double traj_width = xmax - xmin;
      final double traj_height = ymax - ymin;
      final double traj_size = Math.max(traj_width, traj_height);
      // Scale to fit onto plot allowing for 10 pixels around edges
      final double plot_size = Math.min(getWidth(), getHeight()) - 20;
      final double scale = plot_size / traj_size;

      final double total_time = trajectory.getTotalTimeSeconds();
      for (double time=0; time < total_time+1;  time += time_step)
      {
        final State state = trajectory.sample(time);
        final int x = 10 + (int) Math.round((traj_height - state.poseMeters.getTranslation().getY() + ymin) * scale);
        final int y = 10 + (int) Math.round((traj_width  - state.poseMeters.getTranslation().getX()) * scale);
        if (time == 0)
          g.setColor(Color.GREEN); // Start
        else if (time >= total_time)
          g.setColor(Color.RED);   // End
        else
          g.setColor(Color.BLUE);  // On the move
        g.fillOval(x-5, y-5, 10, 10);

        final double speed = state.velocityMetersPerSecond * 20 / speedmax;
        final int x1 = x - (int) Math.round(state.poseMeters.getRotation().getSin() * speed);
        final int y1 = y - (int) Math.round(state.poseMeters.getRotation().getCos() * speed);
        g.drawLine(x, y, x1, y1);
      }
      // Start point tends to get overdrawn by initial waypoints, so draw again on top
      final State state = trajectory.sample(0);
      final int x = 10 + (int) Math.round((traj_height - state.poseMeters.getTranslation().getY() + ymin) * scale);
      final int y = 10 + (int) Math.round((traj_width  - state.poseMeters.getTranslation().getX()) * scale);
      g.setColor(Color.GREEN); // Start
      g.fillOval(x-5, y-5, 10, 10);
    }
  }

  public TrajectoryViewer(final Trajectory trajectory)
  {
    this(trajectory, 0.5);
  }

  public TrajectoryViewer(final Trajectory trajectory, final double time_step)
  {
    this.trajectory = trajectory;
    this.time_step = time_step;
    SwingUtilities.invokeLater(this::createAndShowPlot);
  }

  private void createAndShowPlot()
  {
    final JFrame frame = new JFrame("Trajectory Viewer");
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    frame.getContentPane().add(new JLabel(TrajectoryHelper.getEndInfo(trajectory)),
                               BorderLayout.NORTH);
    
    frame.getContentPane().add(new TrajectoryPlot(),
                               BorderLayout.CENTER);

    frame.getContentPane().add(new JLabel(String.format("Total time: %.2f seconds",
                                                        trajectory.getTotalTimeSeconds())),
                               BorderLayout.SOUTH);
    frame.setBounds(10, 10, 600, 800);
    frame.setVisible(true);
  }

  public static void main(String[] args)
  {
    // Create demo trajectory
    final TrajectoryConfig config = new TrajectoryConfig(1.0, 0.3);
    final Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    Translation2d pos = start.getTranslation();
    
    final List<Translation2d> waypoints = new ArrayList<>();
    pos = pos.plus(new Translation2d(1, 0));
    waypoints.add(pos);
    
    pos = pos.plus(new Translation2d(1, 1));
    waypoints.add(pos);
    
    pos = pos.plus(new Translation2d(1, 0));
    final Pose2d end = new Pose2d(pos, Rotation2d.fromDegrees(0.0));
    final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    // Show it
    new TrajectoryViewer(trajectory);
  }
}