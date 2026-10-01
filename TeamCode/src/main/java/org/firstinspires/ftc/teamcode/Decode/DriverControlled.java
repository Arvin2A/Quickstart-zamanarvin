package org.firstinspires.ftc.teamcode.Decode;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.qualcomm.robotcore.util.ElapsedTime;

import FTC_Classes.Range;

@TeleOp
@Disabled
public class DriverControlled extends OpMode {
    DcMotor fl, fr, bl, br;
    CRServo motorCatapultFar, motorCatapultClose;
    IMU imu;
    boolean left = false;
    boolean right = false;
    private double RELEASE=0.9;
    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();
    private String side="red";
    private double aimbot;
    private double targetX=144.0;
    private double targetY=144.0;
    private double kP=0.45;
    private double kD=0.01;
    private double error=0;
    private double last_error=0;
    private double offset=0;
    private double goalX=0;
    private double curTime=0;
    private double lastTime=0;
    private double[] step_sizes={1.0, 0.1, 0.01, 0.001, 0.0001};
    private int step_index=2;

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
       /*fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;


        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        follower= Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110, 135, Math.toRadians(90)));
        telemetry.addData("Side", side);
        telemetry.update();
        runtime.reset();
    }
    @Override
    public void init_loop(){
        if (gamepad1.b) {
            side="red";
            follower.setStartingPose(new Pose(110, 135, Math.toRadians(0)));
            targetX=144;
            targetY=144;
            offset=-180;
        }
        if (gamepad1.x) {
            side="blue";
            follower.setStartingPose(new Pose(35, 135, Math.toRadians(180)));
            targetX=0;
            targetY=144;
            offset=180;
        }
        telemetry.addData("Side", side);
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.update();
    }
    @Override
    public void loop() {
        double y=-gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x;
        double rx=gamepad1.right_stick_x;
        //Aim
        follower.update();
        Pose currentPose=follower.getPose();
        double xOffset=targetX-currentPose.getX();
        double yOffset=targetY-currentPose.getY();
        double distance=(Math.hypot(
                yOffset,
                xOffset
        ));
        double angleToTarget = Math.atan2(
                yOffset,
                xOffset
        );
        double angle = Math.toDegrees(angleToTarget);
        angle+=90;
        double bot_heading=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (gamepad1.a) {imu.resetYaw();}
        if(gamepad1.x){
            error=AngleUnit.normalizeRadians((Math.toRadians(angle))-bot_heading);
            rx=kP*error;
            last_error=error;
            lastTime=curTime;
        }
        // thing
        drive(y, x, rx);
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
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Heading(RADIANS)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Heading(DEGREES)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Target Heading(RADIANS)", Math.toRadians(aimbot));
        telemetry.addData("Target Heading(DEGREES)", aimbot);
        telemetry.addData("Distance from target", distance);
        telemetry.addData("Pedropathing heading output", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Error", error);
        telemetry.update();
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



