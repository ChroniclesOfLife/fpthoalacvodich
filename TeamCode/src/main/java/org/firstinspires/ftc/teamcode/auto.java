package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "auto")
public class auto extends LinearOpMode {
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double clicksPerDeg = 21.94;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    private static final String[] LABELS = {"Pixel",};
    int lfPos, rfPos, lrPos, rrPos;
    List<AprilTagDetection> currAprilTags;
    List<Recognition> currTfodDets;
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo plane;
    private DcMotor rollIn;
    private DcMotor dualArm;
    private Servo garbageCollector;
    private DistanceSensor distanceL;
    private DistanceSensor distanceR;
    private DistanceSensor distanceB;

    private void moveVertical(int howMuch, double speed) {
        // howMuch is in mm. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = frontLeftMotor.getCurrentPosition();
        rfPos = frontRightMotor.getCurrentPosition();
        lrPos = backLeftMotor.getCurrentPosition();
        rrPos = backRightMotor.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * DRIVE_COUNTS_PER_MM;
        rfPos += howMuch * DRIVE_COUNTS_PER_MM;
        lrPos += howMuch * DRIVE_COUNTS_PER_MM;
        rrPos += howMuch * DRIVE_COUNTS_PER_MM;

        // move robot to new position
        frontLeftMotor.setTargetPosition(lfPos);
        frontRightMotor.setTargetPosition(rfPos);
        backLeftMotor.setTargetPosition(lrPos);
        backRightMotor.setTargetPosition(rrPos);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        // wait for move to complete
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Foward");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void moveHorizontal(int howMuch, double speed) {
        // howMuch is in mm. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = frontLeftMotor.getCurrentPosition();
        rfPos = frontRightMotor.getCurrentPosition();
        lrPos = backLeftMotor.getCurrentPosition();
        rrPos = backRightMotor.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * DRIVE_COUNTS_PER_MM;
        rfPos -= howMuch * DRIVE_COUNTS_PER_MM;
        lrPos -= howMuch * DRIVE_COUNTS_PER_MM;
        rrPos += howMuch * DRIVE_COUNTS_PER_MM;

        // move robot to new position
        frontLeftMotor.setTargetPosition(lfPos);
        frontRightMotor.setTargetPosition(rfPos);
        backLeftMotor.setTargetPosition(lrPos);
        backRightMotor.setTargetPosition(rrPos);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        // wait for move to complete
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }


    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void turn(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        // fetch motor positions
        lfPos = frontLeftMotor.getCurrentPosition();
        rfPos = frontRightMotor.getCurrentPosition();
        lrPos = backLeftMotor.getCurrentPosition();
        rrPos = backRightMotor.getCurrentPosition();

        // calculate new targets
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        frontLeftMotor.setTargetPosition(lfPos);
        frontRightMotor.setTargetPosition(rfPos);
        backLeftMotor.setTargetPosition(lrPos);
        backRightMotor.setTargetPosition(rrPos);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        // wait for move to complete
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void moveHorizontalContinuous(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
    }

    private void moveVerticalContinuous(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    /**
     * Describe this function...
     */
    private void roll() {
        rollIn.setPower(0);
        if (gamepad1.left_bumper) {
            rollIn.setDirection(DcMotorSimple.Direction.FORWARD);
            rollIn.setPower(1);
        }
        if (gamepad1.right_bumper) {
            rollIn.setDirection(DcMotorSimple.Direction.REVERSE);
            rollIn.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private void armUp_Down() {
        dualArm.setPower(0);
        if (gamepad1.a) {
            dualArm.setDirection(DcMotorSimple.Direction.REVERSE);
            dualArm.setPower(0.3);
        }
        if (gamepad1.y) {
            dualArm.setDirection(DcMotorSimple.Direction.FORWARD);
            dualArm.setPower(0.3);
        }
    }


    private void InitVision() {
        aprilTag = new AprilTagProcessor.Builder().build();

        tfod = new TfodProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessors(aprilTag, tfod);
        builder.build();
    }

    private void Telemetry() {
        if (currAprilTags != null) {
            for (AprilTagDetection detection : currAprilTags) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        }

        if (currTfodDets != null) {
            for (Recognition recognition : currTfodDets) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }
        }
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        InitVision();
        double t;
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        rollIn = hardwareMap.get(DcMotor.class, "rollIn");
        dualArm = hardwareMap.get(DcMotor.class, "dualArm");
//        garbageCollector = hardwareMap.get(Servo.class, "garbageCollector");
        distanceL = hardwareMap.get(DistanceSensor.class, "DistanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "DistanceR");
        distanceB = hardwareMap.get(DistanceSensor.class, "DistanceB");
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, false);
//        // Put initialization blocks here.
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            t = getRuntime();
            if (distanceL.getDistance(DistanceUnit.CM) <= 70) {
                moveHorizontal(207,0.6);
                moveVertical();

            }
            else while ((getRuntime() - t) < 2) {
            }






























            /*
            ElapsedTime timer = new ElapsedTime();
            double starttime = timer.time(TimeUnit.SECONDS);
            boolean adu = false;
            boolean isLeft = true;
            double distanceToObject = 0;
            while (!adu && timer.time(TimeUnit.SECONDS) - starttime >= 5) {
                moveHorizontalContinuous(0.15);
                adu = distanceL.getDistance(DistanceUnit.CM) < 70 | distanceR.getDistance(DistanceUnit.CM) < 70;
                isLeft = distanceL.getDistance(DistanceUnit.CM) < distanceR.getDistance(DistanceUnit.CM);
                distanceToObject = isLeft ? distanceL.getDistance(DistanceUnit.CM) : distanceR.getDistance(DistanceUnit.CM);
                distanceToObject = Math.round(distanceToObject);
            }
            stopAllMotors();

            if (!adu) {
                turn(-60, 0.3);
                moveVertical(25, 0.4);
                rollIn.setPower(0.1);
                try {
                    wait(1);
                } catch (InterruptedException e) {
                    rollIn.setPower(0);
                }
                rollIn.setPower(0);

            } else {
                if (isLeft) {
                    moveHorizontal(-150, 0.4);
                } else {
                    moveHorizontal(150, 0.4);
                }

                moveVertical((int) (distanceToObject * 10.0 + 50.0), 0.5);

                rollIn.setPower(0.1);
                try {
                    wait(1);
                } catch (InterruptedException e) {
                    rollIn.setPower(0);
                }
                rollIn.setPower(0);
            }
            moveVertical((int) -(distanceToObject * 10.0 + 50.0), 0.6);
            turn(90, 0.5);

            while (distanceB.getDistance(DistanceUnit.CM) >= 90) {
                moveVerticalContinuous(0.15);
            }
            stopAllMotors();
            CODE CU */

        }
    }
}