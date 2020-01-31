/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.udp;

import java.util.concurrent.atomic.AtomicInteger;

/** Thread that keeps reading from UDPClient, remembering the latest value */
public class UDPReceiverThread
{
  /** Smallest 'int', not expected to be a valid number, used to mark state data */
  public static final int STALE = Integer.MIN_VALUE;

  /** The latest number, 'atomic' because accessed by
   *  a) Receiving thread
   *  b) Thread that checks what we received
   */
  private final AtomicInteger latest = new AtomicInteger(STALE);
  private final UDPClient client;
  private final Thread thread;

  public UDPReceiverThread(final int port) throws Exception
  {
    client = new UDPClient(port);
    thread = new Thread(this::receive);
    thread.setDaemon(true);
    thread.start();
  }

  private void receive()
  {
    try
    {
      while (true)
        latest.set(client.read());
    }
    catch (Exception ex)
    {
      ex.printStackTrace();
    }
    System.err.println("Receive thread quits");
  }

  /** @return Latest value or 'STALE' */
  public int get()
  {
    // Get the latest value and then mark it as stale.
    // When we're called next time, either
    // a) We did receive a new value -> Good
    // b) No new value -> Caller can see that it's STALE
    return latest.getAndSet(STALE);
  }
}
