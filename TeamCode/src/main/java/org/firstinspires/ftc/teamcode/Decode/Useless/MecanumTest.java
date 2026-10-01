package org.firstinspires.ftc.teamcode.Decode.Useless;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp
@Disabled
public class MecanumTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private double RELEASE = 0.9;
    private DcMotor fl, fr, bl, br;
    @Override
    public void runOpMode() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        fl.setZeroPowerBehavior(BRAKE);
        fr.setZeroPowerBehavior(BRAKE);
        bl.setZeroPowerBehavior(BRAKE);
        br.setZeroPowerBehavior(BRAKE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boolean left = false;
        boolean right = false;
        double heading_setpoint=0.0;
        double kP = 1.3;
        double kP2 = 1.0;
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.resetYaw();

        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (gamepad1.x) {
                double error = AngleUnit.normalizeRadians(Math.toRadians(heading_setpoint)) - (Math.toRadians(botHeading));
                if (error > 1) {
                    rx = kP * error;
                } else {
                    rx = kP2 * error;
                }
            }
            if (gamepad1.b) {
                imu.resetYaw();
            }
            if (gamepad1.a) {
                heading_setpoint = (Math.toRadians(botHeading));
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flpower=((y + x + rx) / denominator);
            double blpower=((y - x + rx) / denominator);
            double frpower=((y - x - rx) / denominator);
            double brpower=((y + x - rx) / denominator);
            double maxPower = 1.0;
            double maxSpeed = 1.0;

            maxPower = Math.max(maxPower, Math.abs(flpower));
            maxPower = Math.max(maxPower, Math.abs(frpower));
            maxPower = Math.max(maxPower, Math.abs(brpower));
            maxPower = Math.max(maxPower, Math.abs(blpower));

            fl.setPower(maxSpeed * (flpower / maxPower));
            fr.setPower(maxSpeed * (frpower / maxPower));
            bl.setPower(maxSpeed * (blpower / maxPower));
            br.setPower(maxSpeed * (brpower / maxPower));

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Imu", "Heading: " + botHeading);
            telemetry.addData("Heading", "Setpoint: " + heading_setpoint);
            telemetry.update();
        }
    }
}
