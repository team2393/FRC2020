/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to rotate wheel based on camera info */
public class RotateWheel extends CommandBase
{
    private final ControlWheel wheel;
    private final int required_sectors;

    /** Number of color wheel sectors that we need to see go by */
    private int sectors;

    /** Last pipeline calls that we saw */
    private int last_calls = -1;
    private int next_color = -1;

    /** @param wheel Control wheel to turn
     *  @param times How many times should be turn the wheel?
     */
    public RotateWheel(final ControlWheel wheel, final int times)
    {
        this.wheel = wheel;
        required_sectors = times * 8;
        addRequirements(wheel);
    }

    @Override
    public void initialize()
    {
        sectors = required_sectors;
        next_color = -1;
    }

    @Override
    public void execute()
    {
        // Rotate somewhat slowly in the direction that will
        // give the correct order of colors
        wheel.spin(-0.05);

        // Do we have new image information?
        int calls = (int) SmartDashboard.getNumber("PipelineCalls", -1);
        if (calls == last_calls)
        {
            // System.out.println("Stale image info");
            return;
        }
        last_calls = calls;

        // Did the camera detect a color?
        int color = (int) SmartDashboard.getNumber("Color Idx", -1);
        if (color < 0)
        {
            // System.out.println("Unknown color");
            return;
        }

        // Is this the first time we see a color?
        if (next_color < 0)
        {
            next_color = (color + 1) % ControlWheel.COLORS.length;
            System.out.println("Started on " + ControlWheel.COLORS[color] +
                               ", looking for " + ControlWheel.COLORS[next_color]);
            return;
        }

        // Have we reached the next expected color?
        if (color == next_color)
        {
            // One more sector done
            --sectors;
            if (isFinished())
            {
                System.out.println("Found " + ControlWheel.COLORS[color] +
                                   ", methinks I'm DONE!");
                return;
            }
            next_color = (color + 1) % ControlWheel.COLORS.length;
            System.out.println("Found " + ControlWheel.COLORS[color] +
                               ", now looking for " + ControlWheel.COLORS[next_color] +
                               ", " + sectors + " more sectors");
        }
    }

    @Override
    public void end(final boolean interrupted)
    {
        wheel.spin(0);
    }

    @Override
    public boolean isFinished()
    {
        return sectors <= 0;
    }
}
