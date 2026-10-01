package org.firstinspires.ftc.teamcode.BionicBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DriftBot2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("motor0");
        DcMotor backLeft = hardwareMap.dcMotor.get("motor1");
        DcMotor frontRight = hardwareMap.dcMotor.get("motor3");
        DcMotor backRight = hardwareMap.dcMotor.get("motor2");

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            //x*=-1;
            //y*=-1;
            //rx*=-1;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double maxPower = 1.0;
            double maxSpeed = 1.0;

            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeft.setPower(maxSpeed*-1*(frontLeftPower/maxPower));
            backLeft.setPower(maxSpeed*-1*(backLeftPower/maxPower));
            frontRight.setPower(maxSpeed*(frontRightPower/maxPower));
            backRight.setPower(maxSpeed*(backRightPower/maxPower));

        }
    }
}