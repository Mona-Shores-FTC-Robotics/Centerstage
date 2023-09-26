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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
    public Scalar upperRed1 = new Scalar(12.8, 255, 255);

    public Scalar lowerRed2 = new Scalar(160, 100, 20);
    public Scalar upperRed2 = new Scalar(179, 255, 255);

    public Scalar lowerBlue = new Scalar(58.1, 114.8, 0);
    public Scalar upperBlue = new Scalar(120.4, 255, 255);

    public Scalar lowerStageDoor = new Scalar(97.8, 143.1, 28.3);
    public Scalar upperStageDoor = new Scalar(147.3, 164.3, 110.5);

    public ColorSpace colorSpace = ColorSpace.HSV;

    private Mat hsvMat       = new Mat();

    private Mat maskedRedMat = new Mat();
    private Mat maskedBlueMat = new Mat();
    private Mat maskedStageDoorMat = new Mat();

    private Mat binaryRedMat1      = new Mat();
    private Mat binaryRedMat2      = new Mat();
    private Mat binaryRedMatFinal      = new Mat();
    private Mat binaryBlueMat = new Mat();
    private Mat binaryStageDoorMat = new Mat();

    private Mat leftZoneRed;
    private Mat centerZoneRed;
    private Mat rightZoneRed;
    private Mat leftZoneBlue;
    private Mat centerZoneBlue;
    private Mat rightZoneBlue;

    private Mat leftStageDoorZone;
    private Mat rightStageDoorZone;

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

    public enum AllianceColor {BLUE, RED}

    //set default alliance to Blue
    public AllianceColor allianceColorFinal = AllianceColor.BLUE;

    public enum SideOfField {BACKSTAGE, FRONTSTAGE}

    //set default side of field to BACKSTAGE
    public SideOfField sideOfFieldFinal = SideOfField.BACKSTAGE;

    //set
    public enum StageDoor {LEFT_SIDE, RIGHT_SIDE}
    public StageDoor stageDoorLocation = StageDoor.LEFT_SIDE;

    public enum TeamPropLocation {LEFT, CENTER, RIGHT}
    public TeamPropLocation teamPropLocationFinal = TeamPropLocation.CENTER;

    private int RectLX;
    private int RectLY;
    private int RectMX;
    private int RectMY;
    private int RectRX;
    private int RectRY;

    private int RectLwidth;
    private int RectLheight;
    private int RectMwidth;
    private int RectMheight;
    private int RectRwidth;
    private int RectRheight;

    private int RectLeftSideOfFieldX;
    private int RectLeftSideOfFieldY;
    private int RectLeftSideOfFieldWidth;
    private int RectLeftSideOfFieldHeight;

    private int RectRightSideOfFieldX;
    private int RectRightSideOfFieldY;
    private int RectRightSideOfFieldWidth;
    private int RectRightSideOfFieldHeight;

    private Rect rectL;
    private Rect rectM;
    private Rect rectR;
    private Rect rectLeftSideOfField;
    private Rect rectRightSideOfField;


    private Scalar rectangleColorRed = new Scalar(255, 0, 0);
    private Scalar rectangleColorBlue = new Scalar(0, 0, 255);
    private Scalar rectangleColorGreen = new Scalar(0, 255, 0);
    private Scalar rectangleColorWhite = new Scalar(255, 255, 255);

    private double percentLeftZoneRed;
    private double percentCenterZoneRed;
    private double percentRightZoneRed;

    private double percentLeftZoneBlue;
    private double percentCenterZoneBlue;
    private double percentRightZoneBlue;

    private double percentLeftStageDoorZone;
    private double percentRightStageDoorZone;


    private int TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION = 8;
    private int STAGE_DOOR_THRESHOLD = 15;

    public InitVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        //Set parameters for Left Team Prop Location
        RectLX = 0;
        RectLY = (height*2)/3;
        RectLwidth = width/3;
        RectLheight = height-RectLY;
        rectL = new Rect(RectLX, RectLY, RectLwidth, RectLheight);

        //set parameters for Center Team Prop Location
        RectMX = width/3+2;
        RectMY = (height*2)/3;
        RectMwidth = width/3-2;
        RectMheight = height-RectLY;
        rectM = new Rect(RectMX, RectMY, RectMwidth, RectMheight);

        //set parameters for Right Team Prop Location
        RectRX = (width*2)/3;
        RectRY = (height*2)/3;
        RectRwidth = width/3;
        RectRheight = height-RectLY;
        rectR = new Rect(RectRX, RectRY, RectRwidth, RectRheight);

        //set parameters for Left Side Stage Door Detection
        RectLeftSideOfFieldX = 0;
        RectLeftSideOfFieldY = (height*1)/3;
        RectLeftSideOfFieldWidth = width/3;
        RectLeftSideOfFieldHeight = height/3-2;
        rectLeftSideOfField = new Rect(RectLeftSideOfFieldX, RectLeftSideOfFieldY, RectLeftSideOfFieldWidth, RectLeftSideOfFieldHeight);

        //set parameters for Right Side Stage Door Detection
        RectRightSideOfFieldX = (width*2)/3;
        RectRightSideOfFieldY = (height*1)/3;
        RectRightSideOfFieldWidth = width/3;
        RectRightSideOfFieldHeight = height/3-2;
        rectRightSideOfField = new Rect(RectRightSideOfFieldX, RectRightSideOfFieldY, RectRightSideOfFieldWidth, RectRightSideOfFieldHeight);
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
        Core.inRange(hsvMat, lowerRed1, upperRed1, binaryRedMat1);
        Core.inRange(hsvMat, lowerRed2, upperRed2, binaryRedMat2);
        Core.bitwise_or(binaryRedMat1, binaryRedMat2, binaryRedMatFinal);

        Core.inRange(hsvMat, lowerBlue, upperBlue, binaryBlueMat);

        Core.inRange(hsvMat, lowerStageDoor, upperStageDoor, binaryStageDoorMat);

        Core.bitwise_and(frame, frame, maskedRedMat, binaryRedMatFinal);
        Core.bitwise_and(frame, frame, maskedBlueMat, binaryBlueMat);
        Core.bitwise_and(frame, frame, maskedStageDoorMat, binaryStageDoorMat);

        //this is to check specific zones to determine where the team prop is
        leftZoneRed  = binaryRedMatFinal.submat(rectL);
        centerZoneRed = binaryRedMatFinal.submat(rectM);
        rightZoneRed  = binaryRedMatFinal.submat(rectR);

        leftZoneBlue  = binaryBlueMat.submat(rectL);
        centerZoneBlue = binaryBlueMat.submat(rectM);
        rightZoneBlue  = binaryBlueMat.submat(rectR);

        leftStageDoorZone = binaryStageDoorMat.submat(rectLeftSideOfField);
        rightStageDoorZone = binaryStageDoorMat.submat(rectRightSideOfField);

        //Determine where the team prop is located
        percentLeftZoneRed = ((Core.sumElems(leftZoneRed).val[0] /255) / rectL.area()) * 100 ;
        percentCenterZoneRed = ((Core.sumElems(centerZoneRed).val[0]/255) / rectM.area()) * 100 ;
        percentRightZoneRed= ((Core.sumElems(rightZoneRed).val[0]/255) / rectR.area()) * 100 ;

        percentLeftZoneBlue = ((Core.sumElems(leftZoneBlue).val[0]/255) / rectL.area()) * 100 ;
        percentCenterZoneBlue = ((Core.sumElems(centerZoneBlue).val[0]/255) / rectM.area()) * 100 ;
        percentRightZoneBlue= ((Core.sumElems(rightZoneBlue).val[0]/255) / rectR.area()) * 100 ;

        //Determine whehter stage door is left or right
        percentLeftStageDoorZone = ((Core.sumElems(leftStageDoorZone).val[0] / 255) / rectLeftSideOfField.area()) * 100;
        percentRightStageDoorZone = ((Core.sumElems(rightStageDoorZone).val[0] / 255) / rectRightSideOfField.area()) * 100;

        if (percentLeftZoneRed>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
            percentCenterZoneRed>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
            percentRightZoneRed>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION)
        {
            allianceColorFinal = AllianceColor.RED;
            maskedRedMat.copyTo(frame);
           } else if (percentLeftZoneBlue>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                percentCenterZoneBlue>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION ||
                percentRightZoneBlue>TEAM_PROP_PERCENT_THRESHOLD_FOR_DETECTION)
        {
            allianceColorFinal = AllianceColor.BLUE;
           maskedBlueMat.copyTo(frame);
          }
        else {
            telemetry.addLine("No Alliance Found - defaulting to Blue Alliance");
            maskedStageDoorMat.copyTo(frame);
        }

        if (allianceColorFinal == AllianceColor.RED) {
            //Figure out and Store the Team Prop Location
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

            //Figure out which side of field we are on by combbining alliance and stage door detection
            if (percentLeftStageDoorZone >= percentRightStageDoorZone && percentLeftStageDoorZone > STAGE_DOOR_THRESHOLD) {
                // Stage Door is on the left and we are Red Alliance so we are BACKSTAGE
                sideOfFieldFinal = SideOfField.BACKSTAGE;
            } else if (percentRightStageDoorZone > percentLeftStageDoorZone && percentRightStageDoorZone > STAGE_DOOR_THRESHOLD)
            {
                // Stage Door is on the right and we are Red Alliance so we are FRONTSTAGE
                sideOfFieldFinal = SideOfField.FRONTSTAGE;
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

            //Figure out where the Stage Door is
            if (percentLeftStageDoorZone >= percentRightStageDoorZone && percentLeftStageDoorZone > STAGE_DOOR_THRESHOLD) {
                // Stage Door is on the left and we are Blue Alliance so we are FRONTSTAGE
                sideOfFieldFinal = SideOfField.FRONTSTAGE;
            } else if (percentRightStageDoorZone > percentLeftStageDoorZone && percentRightStageDoorZone > STAGE_DOOR_THRESHOLD)
            {
                // Stage Door is on the right and we are Blue Alliance so we are BACKSTAGE
                sideOfFieldFinal = SideOfField.BACKSTAGE;
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

        //Draw green rectangle to represent where the vision code thinks the prop is located
        if (teamPropLocationFinal == TeamPropLocation.LEFT)
        {
            Imgproc.rectangle(frame, rectL, rectangleColorGreen, 2);
            Imgproc.rectangle(frame, rectM, rectangleColorWhite, 2);
            Imgproc.rectangle(frame, rectR, rectangleColorWhite, 2);
        } else if (teamPropLocationFinal == TeamPropLocation.CENTER)

        {
            Imgproc.rectangle(frame, rectM, rectangleColorGreen, 2);
            Imgproc.rectangle(frame, rectL, rectangleColorWhite, 2);
            Imgproc.rectangle(frame, rectR, rectangleColorWhite, 2);
        } else if (teamPropLocationFinal == TeamPropLocation.RIGHT)
        {
            Imgproc.rectangle(frame, rectR, rectangleColorGreen, 2);
            Imgproc.rectangle(frame, rectL, rectangleColorWhite, 2);
            Imgproc.rectangle(frame, rectM, rectangleColorWhite, 2);
        }

        //Draw colored rectangle where the stage door is to represent
        // we know our alliance color and we know the side of the field we are on
        if (allianceColorFinal == AllianceColor.RED &&  sideOfFieldFinal == SideOfField.BACKSTAGE) {
            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorRed, 2);
            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorWhite, 2);
        } else if (allianceColorFinal == AllianceColor.RED && sideOfFieldFinal == SideOfField.FRONTSTAGE) {
            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorWhite, 2);
            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorRed, 2);

        } else if (allianceColorFinal == AllianceColor.BLUE &&  sideOfFieldFinal == SideOfField.BACKSTAGE) {
            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorWhite, 2);
            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorBlue, 2);

        } else if (allianceColorFinal == AllianceColor.BLUE && sideOfFieldFinal == SideOfField.FRONTSTAGE) {
            Imgproc.rectangle(frame, rectLeftSideOfField, rectangleColorBlue, 2);
            Imgproc.rectangle(frame, rectRightSideOfField, rectangleColorWhite, 2);

        }
        //Release all the mats - is this necessary?
        hsvMat.release();
        maskedRedMat.release();
        maskedBlueMat.release();
        maskedStageDoorMat.release();
        binaryRedMat1.release();
        binaryRedMat2.release();
        binaryRedMatFinal.release();
        binaryBlueMat.release();
        binaryStageDoorMat.release();
        leftZoneRed.release();
        centerZoneRed.release();
        rightZoneRed.release();
        leftZoneBlue.release();
        centerZoneBlue.release();
        rightZoneBlue.release();
        leftStageDoorZone.release();
        rightStageDoorZone.release();


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    public double getLeftPercent() {
        if (allianceColorFinal.equals(AllianceColor.RED)) {
            return percentLeftZoneRed;
        } else if (allianceColorFinal.equals(AllianceColor.BLUE)) {
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