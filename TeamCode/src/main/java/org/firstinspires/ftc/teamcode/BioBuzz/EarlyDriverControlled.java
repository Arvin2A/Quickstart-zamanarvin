package org.firstinspires.ftc.teamcode.BioBuzz;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
@Disabled
public class EarlyDriverControlled extends OpMode {
    private ElapsedTime runtime;
    private DcMotor launcherL, launcherR, intake;
    private DcMotorEx turret;
    private Motor fl, fr, bl, br;
    private MecanumDrive drive;
    private RevIMU imu;
    private GamepadEx driverOp;
    private double targetX=100, targetY=100;
    private double kP=0.2;
    private double kP2=0.3;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(24, 24, 0);
    @Override
    public void init() {
        runtime = new ElapsedTime();
        fl = new Motor(hardwareMap, "frontLeft");
        fr = new Motor(hardwareMap, "frontRight");
        bl = new Motor(hardwareMap, "backLeft");
        br = new Motor(hardwareMap, "backRight");
        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);
        bl.setInverted(true);
        fr.setInverted(false);
        br.setInverted(false);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drive = new MecanumDrive(
                fl, fr, bl, br
        );
        imu = new RevIMU(hardwareMap);
        imu.init();
        driverOp = new GamepadEx(gamepad1);
        launcherL = hardwareMap.dcMotor.get("launcherL");
        launcherR = hardwareMap.dcMotor.get("launcherR");
        intake = hardwareMap.dcMotor.get("intake");
        turret = (DcMotorEx) hardwareMap.dcMotor.get("turret");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherL.setZeroPowerBehavior(FLOAT);
        launcherR.setZeroPowerBehavior(FLOAT);
        launcherL.setDirection(DcMotor.Direction.REVERSE);
        launcherR.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(FLOAT);
        turret.setZeroPowerBehavior(BRAKE);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(110, 135, Math.toRadians(90)));
        telemetry.addData("Glucose Energy: ", "Initialized");
        telemetry.update();
    }
    @Override
    public void start() {runtime.reset();}
    @Override
    public void loop() {
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRotation2d().getDegrees()
        );
        if(gamepad1.left_bumper){
            intake.setPower(1);
        } else if(gamepad1.right_bumper){
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
        follower.manual(-driverOp.getLeftY(), driverOp.getLeftX(), driverOp.getRightX());
        follower.update();
        Pose currentPose = follower.pose();
        track(currentPose, targetX, targetY);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
    public void track(Pose currentPose, double targetX, double targetY) {
        double angleToTarget = Math.atan2(
                targetY - currentPose.y(),
                targetX - currentPose.x()
        );
        double error = angleToTarget - turret.getCurrentPosition();
        turret.setPower(kP * error);
    }
    /*public void track2(Pose currentPose, double targetY, double targetX){
        double angleToTarget = Math.atan2(
                targetY - currentPose.y(),
                targetX - currentPose.x()
        );
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPositionPIDFCoefficients(1);
    }*/
}