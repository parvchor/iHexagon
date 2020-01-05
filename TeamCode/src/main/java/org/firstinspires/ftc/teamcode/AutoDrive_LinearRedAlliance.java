
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;



import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import java.sql.Driver;


@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

public class AutoDrive_LinearRedAlliance extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware         iHexagon   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private int arrangment = 0;

    OpenCvCamera phoneCam;



    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle,  correction;


    @Override
    public void runOpMode() throws InterruptedException {

        iHexagon.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        iHexagon.LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        iHexagon.LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        iHexagon.RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        iHexagon.RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        if(valMid == 0 && valLeft == 255 && valRight == 255)
        {
            arrangment = 1;
        }

        else if(valMid == 255 && valLeft == 255 && valRight == 0)
        {
            arrangment = 2;
        }

        else if(valMid == 255 && valLeft == 0 && valRight == 255)
        {
            arrangment = 3;
        }

        waitForStart();


        while(opModeIsActive())
        {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            switch(arrangment){
                case 1:
                    telemetry.addData("Arrangement:", " yellow      Skystone      yellow");
                    encoderDriveForwardBack(0.75, -16, 1);//move back from starting position
                    encoderDriveStrafeRight(0.6, 40,2);//move to stone quarry
                    encoderDriveForwardBack(0.3,8,3);//move to pick up skystone
                    rotateWheelIntake(1,4,4);//turn wheel intake to pick up skystone
                    encoderDriveForwardBack(0.75,-8,5);//move back after picking up skystone
                    encoderDriveStrafeLeft(1,24.5,6);//move toward middle of alliance side after getting skystone
                    encoderDriveForwardBack(1,-75.5,7);//move toward the foundation zone
                    encoderDriveStrafeRight(0.6,23.5,8);//move toward actual foundation
                    //clamp down on foundation
                    encoderDriveForwardBack(0.5,-23.5/2,9);//go towards triangle
                    rotate(90,0.7,10);//turn foundation
                    encoderDriveStrafeRight(1,6,11);//move foundation to wall
                    // Turn liner slides









                case 2:
                    telemetry.addData("Arrangement:", " yellow      yellow      skystone");


                case 3:
                    telemetry.addData("Arrangement:", " skystone      yellow      yellow");


                case 0:
                    telemetry.addData("Error", "No skystone detected");

            }





            telemetry.update();
            sleep(100);

        }







        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void rotateWheelIntake(double power, double time, int movenum)
    {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time)
        iHexagon.leftWI.setPower(power);
        iHexagon.rightWI.setPower(power);
    }


    public void encoderDriveForwardBack(double speed,
                             double distance, int movenum) {
        iHexagon.LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target  = (int)(COUNTS_PER_INCH * distance);

        iHexagon.LeftF.setTargetPosition(target);
        iHexagon.LeftB.setTargetPosition(target);
        iHexagon.RightF.setTargetPosition(target);
        iHexagon.RightB.setTargetPosition(target);

        iHexagon.LeftF.setPower(speed);
        iHexagon.LeftB.setPower(speed);
        iHexagon.RightF.setPower(speed);
        iHexagon.RightB.setPower(speed);

        iHexagon.LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(iHexagon.LeftF.isBusy() || iHexagon.LeftF.isBusy() || iHexagon.RightB.isBusy() || iHexagon.RightF.isBusy())
        {

        }
        iHexagon.LeftF.setPower(0);
        iHexagon.LeftB.setPower(0);
        iHexagon.RightF.setPower(0);
        iHexagon.RightB.setPower(0);

        telemetry.addLine("On move number" + movenum);
    }

    public void encoderDriveStrafeRight(double speed, double distance, int movenum)
    {
        iHexagon.LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target  = (int)(COUNTS_PER_INCH * distance);

        iHexagon.LeftF.setTargetPosition(target);
        iHexagon.LeftB.setTargetPosition(target);
        iHexagon.RightF.setTargetPosition(target);
        iHexagon.RightB.setTargetPosition(target);

        iHexagon.LeftF.setPower(speed);
        iHexagon.LeftB.setPower(-speed);
        iHexagon.RightF.setPower(-speed);
        iHexagon.RightB.setPower(speed);

        iHexagon.LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(iHexagon.LeftF.isBusy() || iHexagon.LeftF.isBusy() || iHexagon.RightB.isBusy() || iHexagon.RightF.isBusy())
        {

        }
        iHexagon.LeftF.setPower(0);
        iHexagon.LeftB.setPower(0);
        iHexagon.RightF.setPower(0);
        iHexagon.RightB.setPower(0);

        telemetry.addLine("On move number" + movenum);

    }

    public void encoderDriveStrafeLeft(double speed, double distance, int movenum)
    {
        iHexagon.LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iHexagon.RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target  = (int)(COUNTS_PER_INCH * distance);

        iHexagon.LeftF.setTargetPosition(target);
        iHexagon.LeftB.setTargetPosition(target);
        iHexagon.RightF.setTargetPosition(target);
        iHexagon.RightB.setTargetPosition(target);

        iHexagon.LeftF.setPower(-speed);
        iHexagon.LeftB.setPower(speed);
        iHexagon.RightF.setPower(speed);
        iHexagon.RightB.setPower(-speed);

        iHexagon.LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iHexagon.RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(iHexagon.LeftF.isBusy() || iHexagon.LeftF.isBusy() || iHexagon.RightB.isBusy() || iHexagon.RightF.isBusy())
        {

        }
        iHexagon.LeftF.setPower(0);
        iHexagon.LeftB.setPower(0);
        iHexagon.RightF.setPower(0);
        iHexagon.RightB.setPower(0);

        telemetry.addLine("On move number" + movenum);

    }

    private void rotate(int degrees, double power, int movenum)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        iHexagon.LeftB.setPower(leftPower);
        iHexagon.LeftF.setPower(leftPower);
        iHexagon.RightB.setPower(rightPower);
        iHexagon.RightF.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        iHexagon.LeftB.setPower(0);
        iHexagon.LeftF.setPower(0);
        iHexagon.RightB.setPower(0);
        iHexagon.RightF.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();

        telemetry.addLine("On move number" + movenum);


    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }


}
