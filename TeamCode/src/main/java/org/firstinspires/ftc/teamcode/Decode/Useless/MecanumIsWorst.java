/*package org.firstinspires.ftc.teamcode.Decode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.acmerobotics.robotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class MecanumIsWorst extends OpMode {
    Motor fl, fr, bl, br;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    CRServo motorCatapultFar, motorCatapultClose;
    IMU imu;
    boolean left = false;
    boolean right = false;
    private double RELEASE=0.9;

    @Override
    public void init() {
        fl = new Motor(hardwareMap, "front_left");
        fr = new Motor(hardwareMap, "front_right");
        bl = new Motor(hardwareMap, "back_left");
        br = new Motor(hardwareMap, "back_right");

        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fl.setInverted(false);
        bl.setInverted(false);
        fr.setInverted(true);
        br.setInverted(false);
        drive = new MecanumDrive(fl, fr, bl, br);
        motorCatapultFar = hardwareMap.crservo.get("catapult_far");
        motorCatapultClose = hardwareMap.crservo.get("catapult_close");

        motorCatapultFar.setDirection(CRServo.Direction.REVERSE);
        motorCatapultClose.setDirection(CRServo.Direction.FORWARD);
        drive = new MecanumDrive(fl, fr, bl, br);
        driverOp = new GamepadEx(gamepad1);
        /*
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
/*
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
        if (gamepad1.left_bumper){drive.driveRobotCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightY()
        );}
        else {drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );}
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
}
*/