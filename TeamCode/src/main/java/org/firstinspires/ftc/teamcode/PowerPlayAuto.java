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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


/** PSUEDOCODE:
 *
 *
 */


public class PowerPlayAuto {

    private LinearOpMode myOpMode;       // Access to the OpMode object
    private PowerPlayPackBot myRobot;        // Access to the Robot hardware
    private HardwareMap myHardwareMap;
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
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // Class Members
    private OpenGLMatrix location = null;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;

    private final int rows = 640;
    private final int cols = 480;

    private String position = "";
//    OpenCvCamera webcam;

    private double t, tInit;

    public PowerPlayAuto(LinearOpMode theOpMode, PowerPlayPackBot theRobot, HardwareMap theHwMap) {
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
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 1080;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 10.0 / 3.0);
        }
    }

    public void shutdownTFOD() {
        tfod.shutdown();
    }


    public double getHeading() {
        return myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private String scan(String side){
        // Scanner scan = new Scanner(System.in);

        float gain = 5;
        final float[] hsvValues = new float[3];
        String result = "";

        if (side.equals("right")){
            // Turns the light on if it's not on already.
            if (myRobot.colorSensor1 instanceof SwitchableLight) {
                ((SwitchableLight) myRobot.colorSensor1).enableLight(true);
            }

            myRobot.colorSensor1.setGain(gain);

            // Actually gets the colors from the sensor
            NormalizedRGBA colors = myRobot.colorSensor1.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            if (colors.red > colors.blue && colors.red > colors.green){
                result = "Red";
            }
            else if (colors.blue > colors.red && colors.blue > colors.green){
                result = "Blue";
            }
            else if (colors.green > colors.red && colors.green > colors.blue){
                result = "Green";
            }
            else
                result = "Green";
        }
        else if (side.equals("left")){
            // Turns the light on if it's not on already.
            if (myRobot.colorSensor2 instanceof SwitchableLight) {
                ((SwitchableLight) myRobot.colorSensor2).enableLight(true);
            }

            myRobot.colorSensor2.setGain(gain);

            // Actually gets the colors from the sensor
            NormalizedRGBA colors = myRobot.colorSensor2.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            if (colors.red > colors.blue && colors.red > colors.green){
                result = "Red";
            }
            else if (colors.blue > colors.red && colors.blue > colors.green){
                result = "Blue";
            }
            else if (colors.green > colors.red && colors.green > colors.blue){
                result = "Green";
            }
            else
                result = "Green";
        }
        return result;
    }

    public void scanGroundParkRight(){
        /* Move forward enough to read the sleeve
           Scan the sleeve
           Output data about the sleeve
           Drop the cone
           Maybe need to sleep a few milliseconds before and after scan?
           Move depending on the sleeve
        */
        double driveSpeed = 0.4;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String side = "right";

        myRobot.advancedEncoderDrive(myOpMode, 19, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        String result = scan(side);

        myRobot.advancedEncoderDrive(myOpMode, 3, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        //Drop on ground junction
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.groundHeight, liftSpeed);
        myOpMode.sleep(sleepTime);
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        myOpMode.sleep(sleepTime);

        if (result.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 6, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 10, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myRobot.advancedEncoderDrive(myOpMode, 6, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
    }

    public void scanGroundParkLeft(){
        /* Move forward enough to read the sleeve
           Scan the sleeve
           Output data about the sleeve
           Drop the cone
           Maybe need to sleep a few milliseconds before and after scan?
           Move depending on the sleeve
        */
        double driveSpeed = 0.4;
        int sleepTime = 300;
        String side = "left";

        myRobot.advancedEncoderDrive(myOpMode, 19, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        String result = scan(side);

        //Drop on ground junction - need to finish
        myRobot.advancedEncoderDrive(myOpMode, 3, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        myOpMode.sleep(sleepTime);

        if (result.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 6, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 10, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myRobot.advancedEncoderDrive(myOpMode, 6, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
    }

    // Drives left and then forward
    public void parkNoSignal() {
        double driveSpeed = 0.4;
        int sleepTime = 400;

        myRobot.advancedEncoderDrive(myOpMode, 24, "Left", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);
    }
}

