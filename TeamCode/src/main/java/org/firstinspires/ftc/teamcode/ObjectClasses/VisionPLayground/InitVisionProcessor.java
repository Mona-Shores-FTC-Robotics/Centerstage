/*
 * Copyright (c) 2023 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.ObjectClasses.VisionPLayground;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class InitVisionProcessor implements VisionProcessor {

    public Scalar lowerRed1 = new Scalar(0, 100, 20);
    public Scalar upperRed1 = new Scalar(10, 255, 255);

    public Scalar lowerRed2 = new Scalar(160, 100, 20);
    public Scalar upperRed2 = new Scalar(179, 255, 255);

    public Scalar lowerBlue = new Scalar(78, 158, 124);
    public Scalar upperBlue = new Scalar(138, 255, 255);

    public ColorSpace colorSpace = ColorSpace.HSV;

    private Mat hsvMat       = new Mat();

    private Mat maskedRedMat = new Mat();
    private Mat maskedBlueMat = new Mat();

    private Mat binaryRedMat1      = new Mat();
    private Mat binaryRedMat2      = new Mat();
    private Mat binaryRedMatFinal      = new Mat();
    private Mat binaryBlueMat = new Mat();

    private Mat leftZoneRed;
    private Mat centerZoneRed;
    private Mat rightZoneRed;
    private Mat leftZoneBlue;
    private Mat centerZoneBlue;
    private Mat rightZoneBlue;

    private Telemetry telemetry = null;

    enum ColorSpace {
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }


    /** Track Alliance Color, Side of Field, and Team Prop Location with vision**/
    public enum AllianceColor {BLUE, RED}
    public enum SideOfField {BACKSTAGE, FRONTSTAGE}
    public enum TeamPropLocation {LEFT, CENTER, RIGHT}

    //These variables store vision's final decision
    //TODO: make sure these values if determined in auto can be accessed in Teleop

    public static AllianceColor allianceColorFinal = AllianceColor.BLUE;
    public static SideOfField sideOfFieldFinal = SideOfField.BACKSTAGE;
    public static TeamPropLocation teamPropLocationFinal = TeamPropLocation.CENTER.CENTER;


    //TODO: Can we resize these areas of interest and make them dynamic with the resolution?
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

    private final Rect rectL = new Rect(RectLX, RectLY, RectLwidth, RectLheight);
    private final Rect rectM = new Rect(RectMX, RectMY, RectMwidth, RectMheight);
    private final Rect rectR = new Rect(RectRX, RectRY, RectRwidth, RectRheight);
    private final Scalar rectangleColor = new Scalar(170, 255, 0);

    private double percentLeftZoneRed;
    private double percentCenterZoneRed;
    private double percentRightZoneRed;

    private double percentLeftZoneBlue;
    private double percentCenterZoneBlue;
    private double percentRightZoneBlue;

    //TODO: this number needs to be tuned based on reality
    private int TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION = 30;

    public InitVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsvMat, colorSpace.cvtCode);

        /*
         * This is where our thresholding actually happens.
         * Takes our "hsvMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars above (and modifiable with EOCV-Sim's
         * live variable tuner.)
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */

        /** The Red HSV values wrap around so we have to this twice and then use "OR" functionality to put it together **/
        //TODO: This seems to work, but the values for red might need to be tweaked
        Core.inRange(hsvMat, lowerRed1, upperRed1, binaryRedMat1);
        Core.inRange(hsvMat, lowerRed2, upperRed2, binaryRedMat2);
        Core.bitwise_or(binaryRedMat1, binaryRedMat2, binaryRedMatFinal);

        //TODO: These values seem out of whack for blue. Likely need to adjust lowerBlue and upperBlue Scalars
        Core.inRange(hsvMat, lowerBlue, upperBlue, binaryBlueMat);

        //Took this from an example, im not quite sure this part is right
        maskedRedMat.release();
        maskedBlueMat.release();

        /**
         * This takes our original frame capture and does an AND operation with the binary Mat
         *
         * In essence, this means that all the pixels in the original mat are set to zero (0,0,0)
         * except for those that appear white in our thresholding above.
         *
         * This is useful to display to the user, but it might be better to use the masked Mats for doing our calculations
         */

        Core.bitwise_and(frame, frame, maskedRedMat, binaryRedMatFinal);
        Core.bitwise_and(frame, frame, maskedBlueMat, binaryBlueMat);

        //this is to check specific zones to determine where the team prop is
        //using the binary mats that have white pixels.
        leftZoneRed  = binaryRedMatFinal.submat(rectL);
        centerZoneRed = binaryRedMatFinal.submat(rectM);
        rightZoneRed  = binaryRedMatFinal.submat(rectR);

        leftZoneBlue  = binaryBlueMat.submat(rectL);
        centerZoneBlue = binaryBlueMat.submat(rectM);
        rightZoneBlue  = binaryBlueMat.submat(rectR);

        /**
         * Determine where the team prop is located because we are using submats of the binary mats
         * the values stored are white (255,255,255) if they were in range or black (0, 0, 0) if not
         *
         * To determine a percent of the zone that is white, we add up all the pixel values from one channel
         * (should not matter which) and divide by 255, then divide by the area, then multiply by 100.
         *
         * so if we had a 5x5 grid (25 pixels) with 5 white pixels, we would get (255*5/255)/25) *100 == 20%
         */

        percentLeftZoneRed = ((Core.sumElems(leftZoneRed).val[0] / 255) / rectL.area() ) * 100;
        percentCenterZoneRed = ((Core.sumElems(centerZoneRed).val[0] / 255) / rectM.area()) * 100;
        percentRightZoneRed= ((Core.sumElems(rightZoneRed).val[0] / 255 ) / rectR.area()) * 100;

        percentLeftZoneBlue = ((Core.sumElems(leftZoneBlue).val[0] / 255) / rectL.area()) * 100;
        percentCenterZoneBlue = ((Core.sumElems(centerZoneBlue).val[0] / 255) / rectM.area()) * 100;
        percentRightZoneBlue= ((Core.sumElems(rightZoneBlue).val[0] / 255) / rectR.area()) * 100;

        /**
         * How can we determine if we are on the Red or Blue Alliance
         * Step 1: add up the percentages of blue in the three zones and the percentages of red in each zone
         * Step 2: pick alliance based on which has more
         * Step 3: double check against a threshold per zone to make sure there is enough for it to be our prop*
         */

        double sumOfBlueZonePercentages = percentLeftZoneBlue + percentCenterZoneBlue + percentRightZoneBlue;
        double sumOfRedZonePercentages = percentCenterZoneRed + percentCenterZoneRed + percentRightZoneRed;

        if (sumOfRedZonePercentages >= sumOfBlueZonePercentages) {
            if (    percentLeftZoneRed > TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                    percentCenterZoneRed > TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                    percentRightZoneRed > TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION) {
                allianceColorFinal = AllianceColor.RED;
                maskedRedMat.copyTo(frame);

                telemetry.addData("[Percent Red for Left Zone]", percentLeftZoneRed);
                telemetry.addData("[Percent Red for Center Zone]", percentCenterZoneRed);
                telemetry.addData("[Percent Red for Right Zone]", percentRightZoneRed);

            } else
            {
                telemetry.addLine("WARNING - Red Alliance Does not Meet Threshold in one spike area");
                //copy the binary mat instead of the masked mat to help with debugging
                binaryRedMatFinal.copyTo(frame);
            }
        } else {
            if (    percentLeftZoneBlue > TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                    percentCenterZoneBlue > TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                    percentRightZoneBlue > TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION) {
                allianceColorFinal = AllianceColor.BLUE;
                maskedBlueMat.copyTo(frame);

                telemetry.addData("[Percent Blue for Left Zone]", percentLeftZoneBlue);
                telemetry.addData("[Percent Blue for Center Zone]", percentCenterZoneBlue);
                telemetry.addData("[Percent Blue for Right Zone]", percentRightZoneBlue);
            } else
            {
                telemetry.addLine("WARNING - Blue Alliance Does not Meet Threshold in one spike area");
                //copy the binary mat instead of the masked mat to help debug
                binaryRedMatFinal.copyTo(frame);
            }
        }

        if (allianceColorFinal == AllianceColor.RED) {
            if (percentLeftZoneRed > percentCenterZoneRed && percentLeftZoneRed > percentRightZoneRed) {
                // Red Team Prop is on the Left
                teamPropLocationFinal = TeamPropLocation.LEFT;
            } else if (percentCenterZoneRed > percentLeftZoneRed && percentCenterZoneRed > percentRightZoneRed) {
                // Team Prop is in the Middle
                teamPropLocationFinal = TeamPropLocation.CENTER;
            } else if (percentRightZoneRed > percentCenterZoneRed && percentRightZoneRed > percentLeftZoneRed) {
                // Team Prop is on th Right
                teamPropLocationFinal = TeamPropLocation.RIGHT;
            } else {
                telemetry.addLine("Can't figure out where the red prop is located - default to center");
                teamPropLocationFinal = TeamPropLocation.CENTER;
            }
        }

        if (allianceColorFinal == AllianceColor.BLUE) {
            if (percentLeftZoneBlue > percentCenterZoneBlue && percentLeftZoneBlue > percentRightZoneBlue) {
                // Red Team Prop is on the Left
                teamPropLocationFinal = TeamPropLocation.LEFT;
            } else if (percentCenterZoneBlue > percentLeftZoneBlue   && percentCenterZoneBlue > percentRightZoneBlue) {
                // Team Prop is in the Middle
                teamPropLocationFinal = TeamPropLocation.CENTER;
            } else if (percentRightZoneBlue > percentCenterZoneBlue && percentRightZoneBlue > percentLeftZoneBlue) {
                // Team Prop is on the Right
                teamPropLocationFinal = TeamPropLocation.RIGHT;
            } else {
                telemetry.addLine("Can't figure out where the blue prop is located - default to center");
                teamPropLocationFinal = TeamPropLocation.CENTER;
            }
        }

        /*
         * Different from OpenCvPipeline, you cannot return
         * a Mat from processFrame. Therefore, we will take
         * advantage of the fact that anything drawn onto the
         * passed `frame` object will be displayed on the
         * viewport. We will just return null here.
         */

        //frame should be in a helpful state due to processing above

        //Draw Three Rectangles on our thresholded output to represent our zones of interest
        Imgproc.rectangle(frame, rectL, rectangleColor, 2);
        Imgproc.rectangle(frame, rectM, rectangleColor, 2);
        Imgproc.rectangle(frame, rectR, rectangleColor, 2);

        //Release all the mats - is this necessary?
        hsvMat.release();
        maskedRedMat.release();
        maskedBlueMat.release();
        binaryRedMat1.release();
        binaryRedMat2.release();
        binaryRedMatFinal.release();
        binaryBlueMat.release();
        leftZoneRed.release();
        centerZoneRed.release();
        rightZoneRed.release();
        leftZoneBlue.release();
        centerZoneBlue.release();
        rightZoneBlue.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    public double getLeftPercent() {
        if (allianceColorFinal == AllianceColor.RED) {
            return percentLeftZoneRed;
        } else if (allianceColorFinal == AllianceColor.BLUE) {
            return percentLeftZoneBlue;
        }
        return percentLeftZoneBlue;
    }

    public double getCenterPercent() {
        if (allianceColorFinal == AllianceColor.RED) {
            return percentCenterZoneRed;
        } else if (allianceColorFinal == AllianceColor.BLUE) {
            return percentCenterZoneBlue;
        }
        return percentCenterZoneBlue;
    }

    public double getRightPercent()
    {
        if (allianceColorFinal == AllianceColor.RED) {
            return percentRightZoneRed;
        } else if (allianceColorFinal == AllianceColor.BLUE) {
            return percentRightZoneBlue;
        }
        return percentRightZoneBlue;
    }

    public TeamPropLocation getTeamPropLocationFinal()
    {
        return teamPropLocationFinal;
    }

    public AllianceColor getAllianceColorFinal()
    {
        return allianceColorFinal;
    }

    public SideOfField getSideOfField()
    {
        return sideOfFieldFinal;
    }


}