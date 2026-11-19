package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main", group = "Competition")
public class Main extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private HolonomicDrive drive;
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() {
        drive = new HolonomicDrive(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            drive.update(gamepad1);
            intake.update(gamepad2);

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front Drive", "FL %.2f FR %.2f", drive.frontLeftPower, drive.frontRightPower);
            telemetry.addData("Back Drive", "BL %.2f BR %.2f", drive.backLeftPower, drive.backRightPower);
            telemetry.addData("Intake", intake.isRunning() ? "ON" : "OFF");
            telemetry.update();
        }

        drive.stop();
        intake.stop();
    }
}