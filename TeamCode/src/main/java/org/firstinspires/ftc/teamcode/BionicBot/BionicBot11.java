package org.firstinspires.ftc.teamcode.BionicBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class BionicBot11 extends LinearOpMode {
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
            double rx = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double r = Math.hypot(x, y);
            theta = AngleUnit.normalizeRadians(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double rotX = x * Math.cos(-theta) - rx * Math.sin(-theta);
            double rotRX = x * Math.sin(-theta) + rx * Math.cos(-theta);

            double frontLeftPower = y + rotRX + rotX;
            double backLeftPower = y + rotRX - rotX;
            double frontRightPower = y - rotRX - rotX;
            double backRightPower = y - rotRX + rotX;

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);



        }
    }
}