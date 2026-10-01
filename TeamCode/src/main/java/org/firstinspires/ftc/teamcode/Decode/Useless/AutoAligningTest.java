package org.firstinspires.ftc.teamcode.Decode.Useless;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp
@Disabled
public class AutoAligningTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double RELEASE = 0.9;
    private DcMotor fl, fr, bl, br;
    private CRServo motorCatapultFar, motorCatapultClose;

    private double currentX = 108.0;
    private double currentY = 108.0;
    private final double tick_to_in = ((96.0 / 25.4) * Math.PI) / 2000.0;
    private double parallel_offsetX = 15;
    private double parallel_offsetY = 15;
    private double perp_offsetX = 15;
    private double perp_offsetY = 15;
    private double lastParallelPos = 0;
    private double lastPerpPos = 0;
    private double lastHeading = 0;
    private boolean left = false;
    private boolean right = false;
    private String side = "blue";
    private double targetX = 0;
    private double targetY = 144;
    private double kP = 1.5;

    private IMU imu;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
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

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            side = "red";
            targetX = 144;
            targetY = 144;
        }
        if (gamepad1.x) {
            side = "blue";
            targetX = 0;
            targetY = 144;
        }
        telemetry.addData("Side Selected", side);
    }

    @Override
    public void start() {
        runtime.reset();
        lastParallelPos = fl.getCurrentPosition();
        lastPerpPos = bl.getCurrentPosition();
        lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double currentPar = fl.getCurrentPosition();
        double currentPerp = bl.getCurrentPosition();

        double dPar = (currentPar - lastParallelPos) * tick_to_in;
        double dPerp = (currentPerp - lastPerpPos) * tick_to_in;
        double dHeading = AngleUnit.normalizeRadians(botHeading - lastHeading);

        double actualForward = dPar - (dHeading * parallel_offsetY);
        double actualStrafe = dPerp + (dHeading * perp_offsetX);

        double deltaX = actualForward * Math.cos(botHeading) - actualStrafe * Math.sin(botHeading);
        double deltaY = actualForward * Math.sin(botHeading) + actualStrafe * Math.cos(botHeading);

        currentX += deltaX;
        currentY += deltaY;

        lastParallelPos = currentPar;
        lastPerpPos = currentPerp;
        lastHeading = botHeading;

        if (gamepad1.b) { imu.resetYaw(); }

        if (gamepad1.a) {
            double angleToGoal = Math.atan2(targetY - currentY, targetX - currentX);
            double error = AngleUnit.normalizeRadians(angleToGoal - botHeading);
            rx = (Math.abs(error) > 0.05) ? error * kP : 0;
        }

        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);
        theta = AngleUnit.normalizeRadians(theta - botHeading);
        double rotY = r * Math.sin(theta);
        double rotX = r * Math.cos(theta) * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        fl.setPower((rotY + rotX + rx) / denominator);
        bl.setPower((rotY - rotX + rx) / denominator);
        fr.setPower((rotY - rotX - rx) / denominator);
        br.setPower((rotY + rotX - rx) / denominator);

        launch();

        telemetry.addData("Side", side);
        telemetry.addData("X Position", "%.2f", currentX);
        telemetry.addData("Y Position", "%.2f", currentY);
        telemetry.addData("Heading", "%.2f deg", botHeading);
    }

    private void launch() {
        if (gamepad1.right_trigger > 0.8 && motorCatapultFar.getPower() <= 0) {
            motorCatapultClose.setPower(RELEASE);
            right = true;
        } else if (gamepad1.left_trigger > 0.8 && motorCatapultClose.getPower() <= 0) {
            motorCatapultFar.setPower(RELEASE);
            left = true;
        } else {
            motorCatapultFar.setPower(0);
            motorCatapultClose.setPower(0);
        }
    }
}