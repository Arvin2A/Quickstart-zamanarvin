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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
@Disabled
public class FlywheelTuner extends OpMode {
    private ElapsedTime runtime;
    public DcMotorEx launcherL, launcherR, turret;
    public DcMotor intake;
    private Motor fl, fr, bl, br;
    private MecanumDrive drive;
    private RevIMU imu;
    private GamepadEx driverOp;
    private double targetX=100, targetY=100;
    private double kP=0.2;
    private double kP2=0.3;
    double flyF=0;
    double flyP=0;
    private double[] step_sizes={10, 1, 0.1, 0.01, 0.001, 0.0001};
    int step_index = 1;
    private double high_vel = 1500;
    private double low_vel = 900;
    private double velocity = low_vel;
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
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        intake = hardwareMap.dcMotor.get("intake");
        turret = (DcMotorEx) hardwareMap.dcMotor.get("turret");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launcherL.setZeroPowerBehavior(FLOAT);
        //launcherR.setZeroPowerBehavior(FLOAT);
        launcherL.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherR.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
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
            intake.setPower(-1);
        } else if(gamepad1.right_bumper){
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        if(gamepad1.y){
            if(velocity==high_vel){
                velocity = low_vel;
            } else{
                velocity =  high_vel;
            }
        } if(gamepad1.b){
            step_index = (step_index+1) % step_sizes.length;
        }
        if(gamepad1.dpad_left){
            flyF -= step_sizes[step_index];
        } else if(gamepad1.dpad_right){
            flyF += step_sizes[step_index];
        } else if(gamepad1.dpad_up){
            flyP += step_sizes[step_index];
        } else if(gamepad1.dpad_down){
            flyP -= step_sizes[step_index];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flyP, 0.0, 0.0, flyF);
        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pidfCoefficients));
        launcherL.setVelocity(velocity);
        launcherR.setVelocity(velocity);

        follower.manual(-driverOp.getLeftY(), driverOp.getLeftX(), driverOp.getRightX());
        follower.update();
        Pose currentPose = follower.pose();
        telemetry.addData("Glucose Energy: ", "Running");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}