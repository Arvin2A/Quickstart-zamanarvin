package org.firstinspires.ftc.teamcode.BioBuzz;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Axon Mini Servo Test", group = "Linear OpMode")
public class HoodTest extends LinearOpMode {

    public Servo theservo = null;

    public double[] stepSizes = {0.01, 0.05, 0.1, 0.5, 1};

    public int currentPositionChangeAmount = 0;


    @Override
    public void runOpMode() {
        theservo = hardwareMap.get(Servo.class, "hoodservo");

        theservo.scaleRange(0.0, 1.0);


        telemetry.addData("status", "servo initialized");
        telemetry.update();

        waitForStart();

        theservo.setPosition(0.0);

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                //increase pos
                theservo.setPosition(theservo.getPosition() + stepSizes[currentPositionChangeAmount]);
            } else if (gamepad1.right_bumper) {
                theservo.setPosition(theservo.getPosition() + stepSizes[currentPositionChangeAmount]);
            } else if (gamepad1.x) {
                currentPositionChangeAmount += 1;
                if (currentPositionChangeAmount >= 5) {
                    currentPositionChangeAmount = 0;
                }
            }
        }
    }
}