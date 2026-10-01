package org.firstinspires.ftc.teamcode.Decode.Tests;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class MecanumIsBad extends OpMode {
    DcMotor fl, fr, bl, br;
    CRServo motorCatapultFar, motorCatapultClose;
    IMU imu;
    boolean left = false;
    boolean right = false;
    private double RELEASE=0.9;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "front_left");
        fr = hardwareMap.get(DcMotor.class, "front_right");
        bl = hardwareMap.get(DcMotor.class, "back_left");
        br = hardwareMap.get(DcMotor.class, "back_right");
        motorCatapultFar = hardwareMap.crservo.get("catapult_far");
        motorCatapultClose = hardwareMap.crservo.get("catapult_close");
        fl.setZeroPowerBehavior(BRAKE);
        fr.setZeroPowerBehavior(BRAKE);
        bl.setZeroPowerBehavior(BRAKE);
        br.setZeroPowerBehavior(BRAKE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        motorCatapultFar.setDirection(CRServo.Direction.REVERSE);
        motorCatapultClose.setDirection(CRServo.Direction.FORWARD);
        /*
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        if (gamepad1.a) {imu.resetYaw();}
        if (gamepad1.left_bumper) {drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);}
        else {driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);}
        if (gamepad1.right_trigger > 0.8 && (!(motorCatapultFar.getPower()>0))){
            motorCatapultClose.setPower(RELEASE);
            right=true;
            left=false;
        } else if(gamepad1.left_trigger>0.8 && (!(motorCatapultClose.getPower()>0))){
            motorCatapultFar.setPower(RELEASE);
            left=true;
            right=false;
        }else{
            motorCatapultFar.setPower(0);
            motorCatapultClose.setPower(0);
            left=false;
            right=false;
        }
    }
    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;
        backLeftPower*=1.2;
        backRightPower*=1.2;
        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        fl.setPower(maxSpeed * (frontLeftPower / maxPower));
        fr.setPower(maxSpeed * (frontRightPower / maxPower));
        bl.setPower(maxSpeed * (backLeftPower / maxPower));
        br.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
