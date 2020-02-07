/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.led.LEDStrip;

/** Robot code for testing devices */
public class TestRobot extends BasicRobot
{
  // private final DigitalInput ball = new DigitalInput(8);
  // private final LEDStrip led = new LEDStrip();

  private final TalonFX talon = new TalonFX(1);
  private final Orchestra orch = new Orchestra(List.of(talon));

  @Override
  public void disabledPeriodic()
  {
  }

  final int[] notes =
  { 
    300,
      0,
      0,
    300,
      0,
    300,
      0,
    600,
    600,
    600,
      0,
      0,
      0,
      0
  };

  private Timer timer = new Timer();

  @Override
  public void teleopInit()
  {
    super.teleopInit();
    timer.start();
  }

  @Override
  public void teleopPeriodic()
  {
    int note = (int) (timer.get() / 0.1);
    if (note >= notes.length)
    {
      note = 0;
      timer.reset();
    }
    talon.set(TalonFXControlMode.MusicTone, notes[note]);
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
    System.out.println(orch.loadMusic("tune.chrp"));
    System.out.println(orch.play());
  }

  @Override
  public void autonomousPeriodic()
  {
  }
}
