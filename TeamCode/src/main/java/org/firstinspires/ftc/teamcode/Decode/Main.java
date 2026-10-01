package org.firstinspires.ftc.teamcode.Decode;

import RobotUtilities.MovementVars;
import Debugging.ComputerDebugging;
import ReturnTypes.FloatPoint;
import Debugging.UdpServer;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class Main {


    public static void main(String[] args) {
        new Main().run();
    }

    /**
     * The program runs here
     */
    public void run(){
        //this is a test of the coding
        ComputerDebugging computerDebugging = new ComputerDebugging();

        ComputerDebugging.clearLogPoints();


        long startTime = System.currentTimeMillis();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(true){

            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            ComputerDebugging.sendRobotLocation(50, 50, 1);
            ComputerDebugging.sendKeyPoint(new FloatPoint(50, 50));
            ComputerDebugging.markEndOfUpdate();
        }
    }




}
