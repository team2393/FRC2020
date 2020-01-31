/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.udp;

import java.time.LocalTime;
import java.util.concurrent.TimeUnit;

/** Receive numbers via UDP in a dedicated thread,
 *  then check at 20Hz if we got an update
 */
public class UDPClient20Hz
{
  public static void main(String[] args) throws Exception
  {
    // Start the receiver thread
    final UDPReceiverThread receiver = new UDPReceiverThread(5801);

    // Back on this thread, i.e. concurrently ..
    while (true)
    {
      // Check at ~20 Hz what we received
      TimeUnit.MILLISECONDS.sleep(1000/20);
      final int number = receiver.get();

      // Is it good data?
      if (number == UDPReceiverThread.STALE)
        System.out.println("--- STALE DATA, YUCK !!!!!!!");
      else
        System.out.println(LocalTime.now() + " " + number);
    }
  }
}

