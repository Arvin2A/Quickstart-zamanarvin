package org.firstinspires.ftc.teamcode.Decode.Tests;

import org.firstinspires.ftc.teamcode.Decode.UdpServer;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.text.DecimalFormat;

import Debugging.ComputerDebugging;

public class Test4 {

    private ElapsedTime runtime = new ElapsedTime();
    private InetAddress address;
    private DatagramSocket udpSocket;
    private double last_udp_time = 0;
    private String msg;
    private static UdpServer udpServer;
    private static StringBuilder messageBuilder=new StringBuilder();
    private static DecimalFormat df = new DecimalFormat("#.00");
    /*
    public static void main() {
        try {
            udpSocket = new DatagramSocket();
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }

        try {
            address = InetAddress.getByName("192.168.43.71"); // Your laptop IP
        } catch (UnknownHostException e) {
            throw new RuntimeException(e);
        }
        UdpServer.kill=false;
        udpServer=new UdpServer(33484);
        Thread runner = new Thread(udpServer);
        runner.start();
        runtime.reset();
    }*/
    public static void main(String[] args) {
        ComputerDebugging.sendRobotLocation(100, 100, -3);
        ComputerDebugging.markEndOfUpdate();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
