package org.firstinspires.ftc.teamcode.BioBuzz;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.pedro.Constants;


//the point is there is no pedropathing stuff here
@TeleOp
@Disabled
public class quickdrive extends OpMode {
    private ElapsedTime runtime;
    private DcMotor launcherL, launcherR, intake;
    private DcMotorEx turret;
    private Motor fl, fr, bl, br;
    private MecanumDrive drive;

    private boolean initializedOther = false;
    private boolean initializeLimelight = false;
    private RevIMU imu;
    private GamepadEx driverOp;
    private double targetX=100, targetY=100;
    private double kP=0.2;
    private double kP2=0.3;

    private final boolean otherEnabled = true;

    private boolean initializeLauncherAndTurretBeta() {
        if (!otherEnabled) return false;
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
        return true;
    }
    private boolean initLimelight() {
        if (!initializeLimelight) return false;
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(100);
        limelight3A.start();
        // TODO: Vision Loop - Call inside while (opModeIsActive())
        // 1. Grab latest frame: LLResult result = limelight3A.getLatestResult();
        // 2. Check target: if (result != null && result.isValid()) {
        // 3. Pull 2D offsets: double tx = result.getTx(); double ty = result.getTy();
        // 4. Pull 3D pose: Pose3D botpose = result.getBotpose(); // Use getBotpose_MT2() for MegaTag2

        return true;
    }
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
        initializedOther = initializeLauncherAndTurretBeta();

        telemetry.addData("early drive ctrl: ", "Initialized");
        telemetry.update();
    }
    @Override
    public void start() {runtime.reset();}
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRotation2d().getDegrees()
        );
        //other
        if (initializedOther) {
            if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        }
    }
    /*public void track(Pose currentPose, double targetX, double targetY) {
        double angleToTarget = Math.atan2(
                targetY - currentPose.y(),
                targetX - currentPose.x()
        );
        double error = angleToTarget - turret.getCurrentPosition();
        turret.setPower(kP * error);
    }
    public void track2(Pose currentPose, double targetY, double targetX){
        double angleToTarget = Math.atan2(
                targetY - currentPose.y(),
                targetX - currentPose.x()
        );
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPositionPIDFCoefficients(1);
    }*/
}