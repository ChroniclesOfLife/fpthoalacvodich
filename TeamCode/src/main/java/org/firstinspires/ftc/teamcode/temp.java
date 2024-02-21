package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.text.DecimalFormat;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class temp extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo plane;
    private DcMotor rollIn;
    private DcMotor dualArm;
    private Servo garbageCollector;

    private IMU imu;
    int flag2;
    int flag1;
    int prevValue;
    double y, x, rx;
    DecimalFormat d = new DecimalFormat("#.##");
    /**
     * Describe this function...
     */

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        plane = hardwareMap.get(Servo.class, "plane");
        rollIn = hardwareMap.get(DcMotor.class, "rollIn");
        dualArm = hardwareMap.get(DcMotor.class, "dualArm");
        garbageCollector = hardwareMap.get(Servo.class, "collect");
        dualArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // Put initialization blocks here.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            flag1 = 0;
            prevValue = 0;
            while (opModeIsActive()) {
                run();
                roll();
                armUp_Down();
                collectGarbage();
                checkAlternation();
                if (flag1 / 2 >= 5) {
                    releasePlane();
                } else {
                    plane.setPosition(0);
                }
                telemetry.addData("flag1", flag1);
                telemetry.addData("flag2", flag2);
                telemetry.addData("", "");
                telemetry.addData("frontLeft", d.format(frontLeftMotor.getPower()));
                telemetry.addData("frontRight", d.format(frontRightMotor.getPower()));
                telemetry.addData("backLeft", d.format(backLeftMotor.getPower()));
                telemetry.addData("backRight", d.format(backRightMotor.getPower()));
                telemetry.addData("", "");
                telemetry.addData("y", d.format(y));
                telemetry.addData("x", d.format(x));
                telemetry.addData("rx", d.format(rx));
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void run() {
        if (Math.abs(gamepad1.left_stick_y) > 0.2 || gamepad1.left_stick_y == 0) {
        }
        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        if (Math.abs(gamepad1.left_stick_x) > 0.2 || gamepad1.left_stick_x == 0) {
            x = gamepad1.left_stick_x; // Counteract imperfect strafin
        }
        if (Math.abs(gamepad1.right_stick_x) > 0.2 || gamepad1.right_stick_x == 0) {
            rx = gamepad1.right_stick_x / 1.5;
        }
        if (gamepad1.options) imu.resetYaw();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    private void armUp_Down() {
        dualArm.setPower(0);
        if (gamepad1.a) {
            dualArm.setDirection(DcMotorSimple.Direction.REVERSE);
            dualArm.setPower(0.6);
        }
        if (gamepad1.y) {
            dualArm.setDirection(DcMotorSimple.Direction.FORWARD);
            dualArm.setPower(0.6);
        }
    }

    /**
     * Describe this function...
     */
    private void roll() {
        rollIn.setPower(0);
        if (gamepad1.left_bumper) {
            rollIn.setDirection(DcMotorSimple.Direction.REVERSE);
            rollIn.setPower(1);
        }
        if (gamepad1.right_bumper) {
            rollIn.setDirection(DcMotorSimple.Direction.FORWARD);
            rollIn.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private void releasePlane() {
        if (gamepad1.b) {
            plane.setPosition(1);
        } else {
            plane.setPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    private void collectGarbage() {
        if (gamepad1.left_trigger > 0) {
            garbageCollector.setPosition(0);
        }
        if (gamepad1.right_trigger > 0) {
            garbageCollector.setPosition(1);
        }
    }

    /**
     * Describe this function...
     */
    private void checkAlternation() {
        if (gamepad1.b) {
            flag2 = 1;
        } else {
            flag2 = 0;
        }
        if (flag2 != prevValue) {
            flag1 += 1;
            prevValue = flag2;
        }
    }
}