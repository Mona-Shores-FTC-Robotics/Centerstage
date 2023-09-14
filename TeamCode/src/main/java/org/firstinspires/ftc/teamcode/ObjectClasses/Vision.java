package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision {

    public OpenCvCamera webcam;
    public int TeamPropLocation;
    public HardwareMap hwMap = null;
    public Mat outPut = new Mat();
    public double LeftMax;
    public double MiddleMax;
    public double RightMax;

    public int channelToExtract = 0;


    //private variables
    private final int RectLX = 5;
    private final int RectLY = 150;
    private final int RectMX = 113;
    private final int RectMY = 150;
    private final int RectRX = 230;
    private final int RectRY = 150;

    private final int RectLwidth = 85;
    private final int RectLheight = 85;
    private final int RectMwidth = 102;
    private final int RectMheight = 85;
    private final int RectRwidth = 80;
    private final int RectRheight = 85;

    private final  Rect rectL = new Rect(RectLX, RectLY, RectLwidth, RectLheight);
    private final Rect rectM = new Rect(RectMX, RectMY, RectMwidth, RectMheight);
    private final Rect rectR = new Rect(RectRX, RectRY, RectRwidth, RectRheight);
    private final Scalar rectanglecolor = new Scalar(0, 0, 0);

    private Mat LeftCrop = new Mat();
    private Mat MiddleCrop = new Mat();
    private Mat RightCrop = new Mat();

    private Core.MinMaxLocResult LeftMinMax;
    private Core.MinMaxLocResult MiddleMinMax;
    private Core.MinMaxLocResult RightMinMax;

    /* Constructor */
    public Vision() {

    }

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new Vision.pipeLine());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    class pipeLine extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {

            input.copyTo(outPut);

            //Draw Three Rectangles
            Imgproc.rectangle(outPut, rectL, rectanglecolor, 2);
            Imgproc.rectangle(outPut, rectM, rectanglecolor, 2);
            Imgproc.rectangle(outPut, rectR, rectanglecolor, 2);
            LeftCrop = input.submat(rectL);
            MiddleCrop = input.submat(rectM);
            RightCrop = input.submat(rectR);

            // channel 2 = red
            // channel 1 = green
            // channel 0 = blue


            Core.extractChannel(LeftCrop, LeftCrop, channelToExtract);
            Core.extractChannel(MiddleCrop, MiddleCrop, channelToExtract);
            Core.extractChannel(RightCrop, RightCrop, channelToExtract);

            LeftMinMax = Core.minMaxLoc(LeftCrop);
            MiddleMinMax = Core.minMaxLoc(MiddleCrop);
            RightMinMax = Core.minMaxLoc(RightCrop);

            LeftMax = LeftMinMax.maxVal;
            MiddleMax = MiddleMinMax.maxVal;
            RightMax = RightMinMax.maxVal;

            if (LeftMax > MiddleMax && LeftMax > RightMax){
            // team element = left
            TeamPropLocation = 0;
             }

            if (MiddleMax > LeftMax && MiddleMax > RightMax){
                // team element = Middle
                TeamPropLocation = 1;
            }

            if (RightMax > MiddleMax && RightMax > LeftMax){
                // team element = Right
                TeamPropLocation = 2;
            }
            Core.extractChannel(outPut, outPut, channelToExtract);
            return outPut;


        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
