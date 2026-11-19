package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HolonomicDrive {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;

    public double frontLeftPower;
    public double frontRightPower;
    public double backLeftPower;
    public double backRightPower;

    public HolonomicDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stop();
    }

    public void update(Gamepad gamepad) {
        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double yaw = gamepad.right_stick_x;

        drive(axial, lateral, yaw);
    }

    public void drive(double axial, double lateral, double yaw) {
        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower = axial - lateral + yaw;
        backRightPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stop() {
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double getLeftRpm() {
        return (Math.abs(frontLeft.getVelocity(AngleUnit.DEGREES) / 6.0)
                + Math.abs(backLeft.getVelocity(AngleUnit.DEGREES) / 6.0)) / 2.0;
    }

    public double getRightRpm() {
        return (Math.abs(frontRight.getVelocity(AngleUnit.DEGREES) / 6.0)
                + Math.abs(backRight.getVelocity(AngleUnit.DEGREES) / 6.0)) / 2.0;
    }

    public double getAverageRpm() {
        return (getLeftRpm() + getRightRpm()) / 2.0;
    }
}
