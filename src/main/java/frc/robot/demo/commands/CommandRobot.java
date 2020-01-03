/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.demo.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BasicRobot;

/** Demo of 'new' commands
 *
 *  Blinks some output in auto
 */
public class CommandRobot extends BasicRobot
{
    private final DigitalOutput led = new DigitalOutput(1);
    private final Command blink = new Blink(led, 500);

    @Override
    public void autonomousInit()
    {
        super.autonomousInit();
        blink.schedule();
    }
}
