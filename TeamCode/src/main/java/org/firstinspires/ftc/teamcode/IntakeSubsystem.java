package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private final DcMotor intake;
    private boolean running;
    private boolean previousTogglePressed;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        stop();
    }

    public void update(Gamepad gamepad) {
        boolean togglePressed = gamepad.a;
        if (togglePressed && !previousTogglePressed) {
            running = !running;
            intake.setPower(running ? 1.0 : 0.0);
        }
        previousTogglePressed = togglePressed;
    }

    public void stop() {
        running = false;
        intake.setPower(0.0);
    }

    public boolean isRunning() {
        return running;
    }
}
