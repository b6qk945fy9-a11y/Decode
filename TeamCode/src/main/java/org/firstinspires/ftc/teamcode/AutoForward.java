package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto Forward 3 Seconds", group = "Competition")
public class AutoForward extends LinearOpMode {
    private static final double FORWARD_POWER = 0.1;
    private static final double FORWARD_TIME_SECONDS = 2.0;
    private static final double STOP_TIME_SECONDS = 2.0;
    private static final double REVERSE_POWER = -0.1;
    private static final double HEADING_KP = 0.02;
    private static final double RPM_KP = 0.002;
    private static final double YAW_DEADBAND_DEGREES = 2.0;
    private static final double RPM_STOP_THRESHOLD = 20.0;
    private static final double YAW_RATE_STOP_THRESHOLD = 2.0;
    private static final double STOP_CONFIRM_SECONDS = 0.30;

    private IMU imu;

    @Override
    public void runOpMode() {
        HolonomicDrive drive = new HolonomicDrive(hardwareMap);
        initializeImu();
        ElapsedTime timer = new ElapsedTime();
        double startingHeading = getHeading();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < FORWARD_TIME_SECONDS) {
            driveStraight(drive, FORWARD_POWER, startingHeading);

            telemetry.addData("Status", "Driving forward");
            telemetry.addData("Time", "%.1f / %.1f seconds", timer.seconds(), FORWARD_TIME_SECONDS);
            telemetry.addData("Left/Right RPM", "%.1f / %.1f", drive.getLeftRpm(), drive.getRightRpm());
            telemetry.update();
        }

        drive.stop();

        telemetry.addData("Status", "Stopped for 2 seconds");
        telemetry.update();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < STOP_TIME_SECONDS) {
            sleep(20);
        }

        timer.reset();
        ElapsedTime stoppedTimer = new ElapsedTime();
        stoppedTimer.reset();

        while (opModeIsActive()) {
            driveStraight(drive, REVERSE_POWER, startingHeading);

            double avgRpm = drive.getAverageRpm();
            double yawRate = Math.abs(getYawRate());

            if (timer.seconds() >= 0.5
                    && avgRpm < RPM_STOP_THRESHOLD
                    && yawRate < YAW_RATE_STOP_THRESHOLD) {
                if (stoppedTimer.seconds() >= STOP_CONFIRM_SECONDS) {
                    break;
                }
            } else {
                stoppedTimer.reset();
            }

            telemetry.addData("Status", "Moving backward to a stop");
            telemetry.addData("RPM", "%.1f", avgRpm);
            telemetry.addData("Yaw rate", "%.1f deg/s", yawRate);
            telemetry.update();
        }

        drive.stop();
    }

    private void initializeImu() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private double getYawRate() {
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return velocity.zRotationRate;
    }

    private void driveStraight(HolonomicDrive drive, double axialPower, double startingHeading) {
        double headingError = getHeading() - startingHeading;
        while (headingError > 180.0) {
            headingError -= 360.0;
        }
        while (headingError < -180.0) {
            headingError += 360.0;
        }

        double headingCorrection = -headingError * HEADING_KP;

        double leftRpm = drive.getLeftRpm();
        double rightRpm = drive.getRightRpm();
        double rpmCorrection = (rightRpm - leftRpm) * RPM_KP;

        double yaw = headingCorrection + rpmCorrection;
        if (yaw > YAW_DEADBAND_DEGREES * HEADING_KP) {
            yaw = Math.min(yaw, 0.25);
        } else if (yaw < -YAW_DEADBAND_DEGREES * HEADING_KP) {
            yaw = Math.max(yaw, -0.25);
        } else {
            yaw = 0.0;
        }

        drive.drive(axialPower, 0.0, yaw);
    }
}
