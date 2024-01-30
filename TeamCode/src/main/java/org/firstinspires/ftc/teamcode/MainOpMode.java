package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "heehheehhehehe")
public class MainOpMode extends LinearOpMode {
    private AprilTagProcessor ATProc;

    private VisionPortal VisPort;

    private Rev2mDistanceSensor DistSens;

    private void InitVision() {
        ATProc = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagID(true)
                .build();

        ATProc.setDecimation(3);

        VisionPortal.Builder VisBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam1"))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(ATProc);

        VisPort = VisBuilder.build();

    }

    @Override
    public void runOpMode() {
        InitVision();
        DistSens = hardwareMap.get(Rev2mDistanceSensor.class, "DistSens");
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<AprilTagDetection> CurrDetections = ATProc.getDetections();
                telemetry.addData("# AprilTags Detected", CurrDetections.size());
                telemetry.addLine(String.format(Locale.ENGLISH, "DisSensVal: %f", DistSens.getDistance(DistanceUnit.CM)));
                for (AprilTagDetection detection : CurrDetections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }
                telemetry.update();
            }
        }
    }
}

