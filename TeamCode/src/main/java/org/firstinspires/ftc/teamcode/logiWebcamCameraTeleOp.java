package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TeleOp: Logitech Webcam AprilTag detection -> telemetry
 *
 * Hardware config name expected (change to match your robot config):
 *   "Webcam 1" - Logitech C920 (or similar) webcam
 */
@TeleOp(name = "Webcam AprilTag TeleOp", group = "TeleOp")
public class logiWebcamCameraTeleOp extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        telemetry.addLine("Webcam AprilTag TeleOp Ready");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            handleAprilTagTelemetry();

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    /** Builds the AprilTag processor and attaches it to the webcam via VisionPortal */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                // .setLensIntrinsics(fx, fy, cx, cy) // set this for accurate distance/pose data;
                // values are specific to your webcam - see FTC docs for calibration values
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        // Optional: tweak resolution/stream format for performance
        // builder.setCameraResolution(new Size(640, 480));
        // builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        visionPortal = builder.build();
    }

    /** Pulls the latest AprilTag detections and pushes them to telemetry */
    private void handleAprilTagTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addLine("---- Webcam AprilTag Data ----");
        telemetry.addData("Tags Detected", detections.size());

        for (AprilTagDetection detection : detections) {

            if (detection.metadata != null) {
                // Recognized tag (matched against the current tag library)
                telemetry.addLine(String.format("\nTag ID %d (%s)", detection.id, detection.metadata.name));
                telemetry.addData("  Range (in)", "%.1f", detection.ftcPose.range);
                telemetry.addData("  Bearing (deg)", "%.1f", detection.ftcPose.bearing);
                telemetry.addData("  Yaw (deg)", "%.1f", detection.ftcPose.yaw);
                telemetry.addData("  X (in)", "%.1f", detection.ftcPose.x);
                telemetry.addData("  Y (in)", "%.1f", detection.ftcPose.y);
                telemetry.addData("  Z (in)", "%.1f", detection.ftcPose.z);
            } else {
                // Detected but not in the tag library, so no pose data available
                telemetry.addLine(String.format("\nTag ID %d (unknown, no pose data)", detection.id));
                telemetry.addData("  Center", "(%.0f, %.0f)", detection.center.x, detection.center.y);
            }
        }

        if (detections.isEmpty()) {
            telemetry.addLine("No tags in view");
        }
    }
}