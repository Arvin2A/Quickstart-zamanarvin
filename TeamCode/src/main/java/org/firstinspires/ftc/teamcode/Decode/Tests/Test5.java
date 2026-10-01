package org.firstinspires.ftc.teamcode.Decode.Tests;

import Debugging.ComputerDebugging;

public class Test5 {
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