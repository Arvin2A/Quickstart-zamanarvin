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
public class BionicBot12 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("motor0");
        DcMotor backLeft = hardwareMap.dcMotor.get("motor1");
        DcMotor backRight = hardwareMap.dcMotor.get("motor2");
        DcMotor frontRight = hardwareMap.dcMotor.get("motor3");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double rx = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - rx * Math.sin(-botHeading);
            double rotY = rx * Math.sin(-botHeading) + rx * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(y), 1.0);
            double frontLeftPower = (rotY + rotX + y) / denominator;
            double backLeftPower  = (rotY - rotX + y) / denominator;
            double frontRightPower = (rotY - rotX - y) / denominator;
            double backRightPower  = (rotY + rotX - y) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}
