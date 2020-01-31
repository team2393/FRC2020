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

/** Receive a number via UDP */
public class UDPClient
{
  private final DatagramChannel udp;
  private final ByteBuffer buffer = ByteBuffer.allocate(Double.BYTES);

  public UDPClient() throws Exception
  {
    udp = DatagramChannel.open(StandardProtocolFamily.INET);
    udp.configureBlocking(true);
    udp.socket().setBroadcast(true);
    udp.socket().setReuseAddress(true);
    udp.bind(new InetSocketAddress("127.0.0.1", 4812));
  }

  public double read() throws Exception
  {
    buffer.clear();
    udp.receive(buffer);
    buffer.flip();
    return buffer.getDouble();
  }

  public static void main(String[] args) throws Exception
  {
    final UDPClient client = new UDPClient();
    while (true)
    {
      final double number = client.read();
      System.out.println(number);
    }  
  }
}