package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground.InitVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Vision {

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private InitVisionProcessor initVisionProcessor;
    public HardwareMap hwMap = null;

    /* Constructor */
    public Vision() {

    }

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        //aprilTag = new AprilTagProcessor.Builder().build();
        initVisionProcessor = new InitVisionProcessor(telemetry);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(initVisionProcessor)
                .addProcessor(aprilTagProcessor)
                .build();

        visionPortal.setProcessorEnabled(initVisionProcessor, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
     }

    public InitVisionProcessor getInitVisionProcessor()
    {
        return initVisionProcessor;
    }

}
