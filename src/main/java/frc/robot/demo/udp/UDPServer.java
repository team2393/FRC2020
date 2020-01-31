/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.udp;

import java.net.InetSocketAddress;
import java.net.StandardProtocolFamily;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.concurrent.TimeUnit;

/** Send a number via UDP */
public class UDPServer
{
  private final DatagramChannel udp;
  private final ByteBuffer buffer = ByteBuffer.allocate(Double.BYTES);
  private final InetSocketAddress broadcast;

  public UDPServer() throws Exception {
    udp = DatagramChannel.open(StandardProtocolFamily.INET);
    udp.configureBlocking(true);
    udp.socket().setBroadcast(true);
    udp.socket().setReuseAddress(true);

    broadcast = new InetSocketAddress("127.255.255.255", 4812);
  }

  public void send(final double number) throws Exception
  {
    buffer.clear();
    buffer.putDouble(number);
    buffer.flip();
    udp.send(buffer, broadcast);
  }

  public static void main(String[] args) throws Exception
  {
    final UDPServer server = new UDPServer();
    double number = 1.0;
    while (true)
    {
      server.send(number);
      number += 1.1;
      TimeUnit.SECONDS.sleep(1);
    }  
  }
}