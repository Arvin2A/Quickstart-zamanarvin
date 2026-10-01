package org.firstinspires.ftc.teamcode.BionicBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class BionicBot9 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("motor0");
        DcMotor backLeft = hardwareMap.dcMotor.get("motor1");
        DcMotor frontRight = hardwareMap.dcMotor.get("motor3");
        DcMotor backRight = hardwareMap.dcMotor.get("motor2");

        //frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            //x*=-1;
            y*=-1;
            //rx*=-1;

            if (gamepad1.options) {
                imu.resetYaw();
            }
            double theta = Math.atan2(y, x);
            double r = Math.hypot(x, y);
            theta = AngleUnit.normalizeRadians(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double newForward = r*Math.sin(theta);
            double newStrafe = r*Math.cos(theta);
            y=newForward;
            x=newStrafe;
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
            //IMU imu = hwMap.get(IMU.class, "imu");


        }
    }
}