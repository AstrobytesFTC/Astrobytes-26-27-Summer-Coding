package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * TeleOp: Limelight AprilTag detection -> telemetry
 *
 * Hardware config name expected (change to match your robot config):
 *   "limelight" - Limelight3A camera
 */
@TeleOp(name = "Limelight AprilTag TeleOp", group = "TeleOp")
public class limelightAprilTagTeleOp extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // ---------- Limelight ----------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);      // how often the Limelight updates results
        limelight.pipelineSwitch(0);       // set this to whichever pipeline has AprilTag detection enabled
        limelight.start();                 // begin polling for data

        telemetry.addLine("Limelight AprilTag TeleOp Ready");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            handleLimelightTelemetry();

            telemetry.update();
        }

        limelight.stop();
    }

    /** Pulls the latest Limelight result and pushes AprilTag data to telemetry */
    private void handleLimelightTelemetry() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("Limelight: no result yet");
            return;
        }

        if (!result.isValid()) {
            telemetry.addLine("Limelight: no valid targets");
            return;
        }

        telemetry.addLine("---- Limelight AprilTag Data ----");
        telemetry.addData("Targeting Latency (ms)", result.getTargetingLatency());
        telemetry.addData("Capture Latency (ms)", result.getCaptureLatency());

        // tx/ty/ta = horizontal offset, vertical offset, target area (from crosshair)
        telemetry.addData("tx (deg)", "%.2f", result.getTx());
        telemetry.addData("ty (deg)", "%.2f", result.getTy());
        telemetry.addData("ta (%%)", "%.2f", result.getTa());

        // Robot pose in field space, IF you've calibrated Limelight field localization
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addData("Bot X (m)", "%.2f", botpose.getPosition().x);
            telemetry.addData("Bot Y (m)", "%.2f", botpose.getPosition().y);
            telemetry.addData("Bot Yaw (deg)", "%.2f", Math.toDegrees(botpose.getOrientation().getYaw()));
        }

        // Individual AprilTag (fiducial) detections
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("Tags Detected", fiducials.size());

        for (LLResultTypes.FiducialResult tag : fiducials) {
            int id = tag.getFiducialId();
            double tagTx = tag.getTargetXDegrees();
            double tagTy = tag.getTargetYDegrees();
            double tagArea = tag.getTargetArea();

            telemetry.addLine(String.format(
                    "Tag %d -> tx: %.2f  ty: %.2f  area: %.2f",
                    id, tagTx, tagTy, tagArea
            ));

            // Pose of the tag relative to the camera, if you need distance/angle for alignment
            Pose3D tagPoseCamSpace = tag.getTargetPoseCameraSpace();
            if (tagPoseCamSpace != null) {
                double distance = Math.sqrt(
                        Math.pow(tagPoseCamSpace.getPosition().x, 2) +
                                Math.pow(tagPoseCamSpace.getPosition().z, 2)
                );
                telemetry.addLine(String.format("  Tag %d distance: %.2f m", id, distance));
            }
        }
    }
}