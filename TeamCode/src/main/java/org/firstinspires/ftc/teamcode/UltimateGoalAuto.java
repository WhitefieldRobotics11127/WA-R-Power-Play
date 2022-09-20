/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class UltimateGoalAuto {

    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private UltimateGoalPackBot myRobot;        // Access to the Robot hardware
    private HardwareMap         myHardwareMap;
    //private VuforiaTrackables   targets = this.vuforia.loadTrackablesFromAsset("Skystone");        // List of active targets
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    boolean PHONE_IS_PORTRAIT = false;
    String VUFORIA_KEY =
            "Adg0Y9v/////AAABmergYcIrxEPZmeeflCjLz7pIqlTKWre7SqTXe94Qzd8Mdv2CWJzL6Dl8jsSNRH7XEzhINohMrWO0MY4Z0Sm9UYg/lPNTYbfXQSXmTAG2623GHGCogvStqInHKbTqICTPgNJYbe4iGRmcJmvCN1oo+N0+0KzZRaCTHXjqZbUPo430TQNUYELOmdMx5+uT2O1jTx75XZ2MMGcanX0aSMXpzE47V6PMAtXAD11h1CaNB4/dYE+CgkqWTEN/PBKvTYJdCMhNUH6PuENY8q6wkv5aTql0q5ZFNamLN6Vl1PxSgiChwBFMZC53ASMbo606s4TzFgcAD3+AiUZHFk2LmYx088Xj5XkvW1s1DN9KhDR4EYn1";

    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // Class Members
    private OpenGLMatrix location = null;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;

    private final int rows = 640;
    private final int cols = 480;

    private String position = "";
//    OpenCvCamera webcam;

    private double t, tInit;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    public UltimateGoalAuto(LinearOpMode theOpMode, UltimateGoalPackBot theRobot, HardwareMap theHwMap) {
        myOpMode = theOpMode;
        myRobot = theRobot;
        myHardwareMap = theHwMap;
    }

    /**
     * Initialize the Vuforia localization engine. Must be done to init TFOD.
     */
    public void initVuforia() {
        HardwareMap hardwareMap = myHardwareMap;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }



    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        HardwareMap hardwareMap = myHardwareMap;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null)
            tfod.activate();
    }

    public void shutdownTFOD() {
        tfod.shutdown();
    }



    // Camera should be ~13in away from rings
    public int getNumberOfRings() {
        int numRings = 0;
        String label = "";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            int timeout = 0;
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            while (updatedRecognitions == null && timeout < 500000) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                timeout++;
            }
            myOpMode.telemetry.addData("timeout", timeout);
            myOpMode.telemetry.update();

            if (updatedRecognitions != null) {
//                myOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    label = recognition.getLabel();
                    myOpMode.telemetry.addData("label", label);
                    myOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    /*
                    myOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    myOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                     */
                }
                myOpMode.telemetry.update();
            }
        } else {
            myOpMode.telemetry.addData("TFOD NOT INITIALIZED", "");
            myOpMode.telemetry.update();
        }
        if (label.equals("Quad")) {
            numRings = 4;
        } else if (label.equals("Single")) {
            numRings = 1;
        } else {
            numRings = 0;
        }
        return numRings;
    }

    public int getAverageRings(int iterations){

        int count4 = 0;
        int count1 = 0;
        int count0 = 0;

        for (int i = 0; i <= iterations; i++){
            if (getNumberOfRings() == 4)
                count4++;
            else if (getNumberOfRings() == 1)
                count1++;
            else if (getNumberOfRings() == 0)
                count0++;
        }
        if (count4 > count1 && count4 > count0)
            return 4;
        else if (count1 > count4 && count1 > count0)
            return 1;
        else
            return 0;

    }

    public double getHeading() {
        return myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void autoBlue() {
        double heading = getHeading();
        double driveSpeed = .4;
        double rotateSpeed = .3;

        myRobot.droppie.setPosition(1.0);
        myRobot.moveWobbleGoal("close");
        myRobot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        //Drive forward to read rings
        myRobot.advancedEncoderDrive(myOpMode,12, "Forward", driveSpeed);



        myOpMode.sleep(1000);

        // Initialize TFOD and scan rings
        int numRings = getNumberOfRings();
        myOpMode.telemetry.addData("Num Rings", getNumberOfRings());
//        myOpMode.telemetry.addData("Avg rings:", getAverageRings(10));
        myOpMode.telemetry.update();

        myOpMode.sleep(1000);

        myRobot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);

        // Drive forward ~72in to launch line w/ odometry
        myRobot.advancedEncoderDrive(myOpMode,44, "Forward", driveSpeed);

        myOpMode.sleep(1000);

        // Use scanned rings from TFOD to place wobble in different locations
        // psuedocode case A
        if (numRings == 0) {
            myRobot.advancedEncoderDrive(myOpMode, 3, "Forward", driveSpeed);

            myOpMode.sleep(1000);

            //Rotate CW 90deg
            heading = getHeading();
            while (heading > -89.5) {
                myRobot.rotateCW(rotateSpeed);
                heading = getHeading();
            }
            myRobot.driveStop();

            myOpMode.sleep(1000);

            //Pull into box
//            myRobot.encoderDrive(myOpMode, 15, "Backward", .3);

            //Move servos to eject Wobble Goal
            myRobot.moveWobbleGoal("open");
            myOpMode.sleep(1000);

            //Pull away from Wobble Goal
            myRobot.rotisserie.setPosition(0);
            myOpMode.sleep(500);
            myRobot.advancedEncoderDrive(myOpMode, 15, "Forward", driveSpeed);

            //Rotate CCW back to original heading
//            heading = getHeading();
//            while (heading < 0) {
//                myRobot.rotateCCW(.3);
//                heading = getHeading();
//            }

            //move none to park on launch line
        }
        // psuedocode case B
        else if (numRings == 1) {
            myRobot.advancedEncoderDrive(myOpMode, 30, "Forward", driveSpeed);

            myOpMode.sleep(1000);

            //Rotate CCW 90deg
            heading = getHeading();
            while (heading < 90) {
                myRobot.rotateCCW(rotateSpeed);
                heading = getHeading();
            }
            myRobot.driveStop();

            myOpMode.sleep(1000);

            //Pull into box
//            myRobot.advancedEncoderDrive(myOpMode, 5, "Backward", driveSpeed);

//            myOpMode.sleep(1000);

            //Move servos to eject Wobble Goal
            myRobot.moveWobbleGoal("open");
            myRobot.rotisserie.setPosition(0);
            myOpMode.sleep(1000);

            //Pull away from Wobble Goal
            myRobot.advancedEncoderDrive(myOpMode, 3, "Forward", driveSpeed);

            myOpMode.sleep(1000);

            //Rotate CW back to original heading
            heading = getHeading();
            while (heading > 1) {
                myRobot.rotateCW(rotateSpeed);
                heading = getHeading();
            }
            myRobot.driveStop();

            myOpMode.sleep(1000);

            //move ~24in backwards to park on launch line
            myRobot.advancedEncoderDrive(myOpMode, 20, "Backward", driveSpeed);
        }
        // psuedocode case C
        else if (numRings == 4) {
            myRobot.advancedEncoderDrive(myOpMode, 42, "Forward", driveSpeed);

            myOpMode.sleep(1000);

            //Rotate CW 90deg
            heading = getHeading();
            while (heading > -90) {
                myRobot.rotateCW(rotateSpeed);
                heading = getHeading();
            }
            myRobot.driveStop();

            myOpMode.sleep(1000);

            //Pull into box
//            myRobot.advancedEncoderDrive(myOpMode, 5, "Backward", driveSpeed);

//            myOpMode.sleep(1000);

            //Move servos to eject Wobble Goal
            myRobot.moveWobbleGoal("open");

            myOpMode.sleep(1000);

            //Pull away from Wobble Goal
            myRobot.rotisserie.setPosition(0);
            myOpMode.sleep(500);
            myRobot.advancedEncoderDrive(myOpMode, 17, "Forward", driveSpeed);

            myOpMode.sleep(1000);

            myRobot.rotisserie.setPosition(1);
            myOpMode.sleep(1300);

            //Rotate CW back to original heading
            heading = getHeading();
            while (heading > -179) {
                myRobot.rotateCW(rotateSpeed);
                heading = getHeading();
            }
            myRobot.driveStop();

            myOpMode.sleep(1000);

            //move ~48in forward to park on launch line
            myRobot.encoderDrive(myOpMode, 35, "Forward", driveSpeed);
        }
    }

    public void lschallenge() {
        double heading = getHeading();
        double driveSpeed = .5;
        double rotateSpeed = .5;
        double strafeSpeed = .5;
        int waitTime = 500; //ms

        double strt_wg1_dist = 21; //actual dist 24 //distance from start line to first wobble goal
        double wg1_wg2_dist = 27; //actual dist 30 //distance from first wobble goal to second wobble goal
        double strafeDist = 15; //distance the robot strafes to dodge the WGs, approx. half the width of the bot
        /*
        Layout:
        Start line, wg1, wg2, all in a straight line.

        General Path (psuedocode):
        Move forward .5(dist to WG1)
        Strafe Right strafeDist
        Move forward .5(strt_wg1_dist) + .5(wg1_wg2_dist)
        Strafe Left 2*strafeDist
        Move forward wg1_wg2_dist //now past the second wg, so basically reverse previous instructions to come back
        Strafe Right 2*strafeDist
        Move backward wg1_wg2_dist
        Strafe Left 2*strafeDist
        Move backward .5(strt_wg1_dist) + .5(wg1_wg2_dist)
        Strafe right strafeDist
        Move backward .5(dist to WG1)
         */

        myRobot.advancedEncoderDrive(myOpMode, .5*strt_wg1_dist, "Forward", driveSpeed);

        myOpMode.sleep(waitTime);

        myRobot.encoderDrive(myOpMode, strafeDist, "Right", strafeSpeed);

        myOpMode.sleep(waitTime);

        myRobot.advancedEncoderDrive(myOpMode, .8*(.5*strt_wg1_dist + .5*wg1_wg2_dist), "Forward", driveSpeed);

        myOpMode.sleep(waitTime);

        myRobot.encoderDrive(myOpMode, 1.8*strafeDist, "Left", strafeSpeed);

        myOpMode.sleep(waitTime);

        heading = getHeading();
        while (heading < 0) {
            myRobot.rotateCW(rotateSpeed/2);
            heading = getHeading();
        }
        myRobot.driveStop();

        myOpMode.sleep(waitTime);

        myRobot.advancedEncoderDrive(myOpMode, .8*wg1_wg2_dist, "Forward", driveSpeed);

        myOpMode.sleep(waitTime);

        myRobot.encoderDrive(myOpMode, 2*strafeDist, "Right", strafeSpeed);

        myOpMode.sleep(waitTime);

        //make sure angle is correct before continuing
        heading = getHeading();
        while (heading < 0) {
            myRobot.rotateCW(rotateSpeed/2.5);
            heading = getHeading();
        } while (heading > 0) {
            myRobot.rotateCCW(rotateSpeed/2.5);
            heading = getHeading();
        }
        myRobot.driveStop();

        myOpMode.sleep(waitTime);

        myRobot.advancedEncoderDrive(myOpMode, wg1_wg2_dist, "Backward", driveSpeed);

        myOpMode.sleep(waitTime);

        myRobot.encoderDrive(myOpMode, 1.8*strafeDist, "Left", strafeSpeed);

        myOpMode.sleep(waitTime);

        myRobot.advancedEncoderDrive(myOpMode, (.5*strt_wg1_dist + .5*wg1_wg2_dist), "Backward", driveSpeed);

        myOpMode.sleep(waitTime);

        myRobot.encoderDrive(myOpMode, strafeDist, "Right", strafeSpeed);

        myOpMode.sleep(waitTime);

        myRobot.advancedEncoderDrive(myOpMode, .5*strt_wg1_dist, "Backward", driveSpeed);

        myOpMode.sleep(waitTime);

        myRobot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);

        //spin around
        heading = getHeading();
        while (heading < 179) {
            myRobot.rotateCW(rotateSpeed);
            heading = getHeading();
        }
//        myRobot.driveStop();
        myOpMode.sleep(300);
        heading = getHeading();
        while (heading < 0) {
            myRobot.rotateCW(rotateSpeed);
            heading = getHeading();
        }
        myRobot.driveStop();
    }

}