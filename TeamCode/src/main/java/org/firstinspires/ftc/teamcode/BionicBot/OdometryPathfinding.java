package org.firstinspires.ftc.teamcode.BionicBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class OdometryPathfinding extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_drive, right_drive;
    public static double odometry_diameter = 3.5;
    public static double odometry_circumference = odometry_diameter * Math.PI;
    public static double ticks = 4096;
    public static double ticks_to_cm = ticks / odometry_circumference;
    public static double hDistance = 0;
    public static double vDistance = 0;

    private double robotX = 0;
    private double robotY = 0;

    int vLast_ticks = 0;
    int hLast_ticks = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Reset the motor encoders to ensure they start at 0
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to run without encoders
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset distances and positions to zero
        robotX = 0;
        robotY = 0;
        hDistance = 0;
        vDistance = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            int vCurrent_ticks = left_drive.getCurrentPosition();
            int hCurrent_ticks = right_drive.getCurrentPosition();

            // Calculate delta ticks for vertical and horizontal movement
            int vDelta_ticks = vCurrent_ticks - vLast_ticks;
            int hDelta_ticks = hCurrent_ticks - hLast_ticks;

            // Update the last ticks
            vLast_ticks = vCurrent_ticks;
            hLast_ticks = hCurrent_ticks;

            // Convert tick deltas to distances
            double hDelta_distance = (hDelta_ticks / ticks) * odometry_circumference;
            double vDelta_distance = (vDelta_ticks / ticks) * odometry_circumference;

            // Update the total horizontal and vertical distances
            hDistance += hDelta_distance;
            vDistance += vDelta_distance;

            // Calculate the deltas for x and y coordinates
            double xDelta = hDelta_distance * Math.cos(botHeading) - vDelta_distance * Math.sin(botHeading);
            double yDelta = hDelta_distance * Math.sin(botHeading) + vDelta_distance * Math.cos(botHeading);

            // Update the robot's position
            robotX += xDelta;
            robotY += yDelta;

            // Send telemetry data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("IMU", "Heading: " + botHeading);
            telemetry.addData("Vertical Ticks: ", vCurrent_ticks);
            telemetry.addData("Horizontal Ticks: ", hCurrent_ticks);
            telemetry.addData("Total Vertical Distance: ", vDistance);
            telemetry.addData("Total Horizontal Distance: ", hDistance);
            telemetry.addData("Robot X Position: ", robotX);
            telemetry.addData("Robot Y Position: ", robotY);
            telemetry.update();
        }
    }
}
