/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

//import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;


/**
 Whitefield Robotics Ultimate Goal TeleOp Code

 */

@TeleOp(name="UG_Rover", group="Packbot")
//@Disabled
public class UG_Rover extends OpMode {

    /* Declare OpMode members. */
    UltimateGoalPackBot robot = new UltimateGoalPackBot();

    //variables
    double heading;

    double forward, rotate, strafe, direction = 1; //direction: 1 is normal, -1 is reversed
    double gear = .5;

    double currStrafeCt, currStraightCt;

    double front_left, front_right, rear_left, rear_right;

//    double wg_left = robot.wg_left_closed, wg_right = robot.wg_right_closed;

    double intake_motor;
    double conveyor_motor;
    double rsticky;

    boolean sprinting = false;

    double defLiftPwr = 0;
    double liftHoldPower = .15;
    double lifty_speed = 1;
    boolean isHolding = true;

    boolean canLift = false;

    RevBlinkinLedDriver.BlinkinPattern defPattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    private final static int GAMEPAD_LOCKOUT = 300;
    Deadline gamepadRateLimit;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        //robot.resetEncoders(robot);

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        telemetry.addData(">", "Shall we play a game?");
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        robot.droppie.setPosition(1.0);
    }


    @Override
    public void loop() {
//        heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        /** GAMEPAD 1 */

        if (gamepad1.a)
            gear = .7;
        if (gamepad1.b)
            gear = 1;
        if (gamepad1.x)
            gear = .5;
        if (gamepad1.y)
            gear = .25;

        if (gamepadRateLimit.hasExpired() && gamepad1.left_stick_button && !sprinting) {
            sprinting = true;
            gear = 0.90;
            gamepadRateLimit.reset();
        } else if (gamepadRateLimit.hasExpired() && gamepad1.left_stick_button && sprinting) {
            sprinting = false;
            gear = 0.50;
            gamepadRateLimit.reset();
        }

        forward = gear * gamepad1.left_stick_y;
        strafe = gear * -gamepad1.left_stick_x;
        rotate = -gear * gamepad1.right_stick_x;


        front_left = direction * forward + rotate + (direction * strafe);
        front_right = direction * forward - rotate - (direction * strafe);
        rear_left = direction * forward + rotate - (direction * strafe);
        rear_right = direction * forward - rotate + (direction * strafe);


        //thunder = struck;


        //Changes if the robot "front" is in the front or the back of the bot
        if (gamepad1.dpad_up)
            direction = 1;
        if (gamepad1.dpad_down)
            direction = -1;


        /** GAMEPAD 2 */

        if (gamepad2.left_stick_button)
            lifty_speed = .3;
//            isHolding = false;
        if (gamepad2.right_stick_button)
            lifty_speed = 1;
//            isHolding = true;

//        if (isHolding && !(defLiftPwr == liftHoldPower)) {
//            defLiftPwr = liftHoldPower;
////            lifty_speed = 1 - defLiftPwr;
//        } else {
//            defLiftPwr = 0;
////            lifty_speed = 1 - defLiftPwr;
//        }


//        if (gamepad2.dpad_up) { //Also conflicts with ring dropper
//            robot.setWobbleGoalTargetPosition("Upper");
//        } else if (gamepad2.dpad_down) {
//            robot.setWobbleGoalTargetPosition("Lower");
//        }

        robot.topConveyor.setPower(gamepad2.right_stick_x);

        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            canLift = true; //canLift override in case it gets stuck for any reason
        } else if (gamepad2.left_bumper) {
            robot.moveWobbleGoal("open");
        } else if (gamepad2.right_bumper) {
            robot.moveWobbleGoal("close");
        }

        if (gamepad2.dpad_left) {
            robot.rotisserie.setPosition(0);
            canLift = true;
        } else if (gamepad2.dpad_right) {
            robot.rotisserie.setPosition(1);
            canLift = false;
        }

        if (gamepad2.x) {
            robot.grabbie.setPosition(.75);
        } else if (gamepad2.b) {
            robot.grabbie.setPosition(0.5);
        }

        if (gamepad2.dpad_up) {
            robot.droppie.setPosition(1.0);
        } else if (gamepad2.dpad_down) {
            robot.droppie.setPosition(0.0);
        }

        if (Math.abs(gamepad2.left_trigger) > 0.85) {
            robot.flywheels(1.0);
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        } else if (Math.abs(gamepad2.left_trigger) > 0.01) {
            robot.flywheels(gamepad2.left_trigger);
        } else {
            robot.flywheels(0.0);
            robot.blinkin.setPattern(defPattern);
        }

        if (Math.abs(gamepad2.right_trigger) > 0.02) {
            robot.shootie.setPosition(0);
        } else {
            robot.shootie.setPosition(0.5);
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            rsticky = gamepad2.right_stick_y;
            intake_motor = rsticky;
//            conveyor_motor = rsticky;
            robot.intakeStage1(rsticky);
            robot.intakeStage2(rsticky);
        } else {
            rsticky = 0.0;
            intake_motor = rsticky;
//            conveyor_motor = rsticky;
            robot.intakeStage1(rsticky);
            robot.intakeStage2(rsticky);
        }
//        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
//            conveyor_motor = gamepad2.left_stick_y;
//        }


        /** TELEMETRY */

        telemetry.addData("Sprinting", sprinting);

//        currStraightCt = robot.dcMotor1.getCurrentPosition();
//        currStraightCt /= robot.COUNTS_PER_INCH;
//        currStrafeCt = robot.dcMotor3.getCurrentPosition();
//        currStrafeCt /= robot.COUNTS_PER_INCH;
//
//        telemetry.addData("Straight 1 Odometer", "" + currStraightCt + " " + robot.dcMotor1.getCurrentPosition());
//        telemetry.addData("Straight 2 Odometer", "" + currStraightCt + " " + robot.dcMotor2.getCurrentPosition());
//        telemetry.addData("Strafe Odometer", "" + currStrafeCt + " " + robot.dcMotor3.getCurrentPosition());
//
//        telemetry.addData("Straight Odo Method", robot.getStraightEncoderCount());
//        telemetry.addData("Horizontal Odo Method", robot.getHorizontalEncoderCount());
//
//        telemetry.addData("Heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle); //Usually what we call "heading"
//        telemetry.addData("IMU Angle 2", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
//        telemetry.addData("IMU Angle 3", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);

//        telemetry.addData("wg_left", wg_left);
//        telemetry.addData("wg_right", wg_right);

//        telemetry.addData("Pot Voltage", robot.pot.getVoltage());

        telemetry.update();

        //The conveyor motor has been changed to be the motor that lifts up the wobble goal
        if (canLift) conveyor_motor = -gamepad2.left_stick_y * lifty_speed; //(gamepad2.left_stick_y * lifty_speed) - defLiftPwr;

        robot.dcMotor1.setPower(front_left);
        robot.dcMotor2.setPower(front_right);
        robot.dcMotor3.setPower(rear_left);
        robot.dcMotor4.setPower(rear_right);

        robot.dcMotor5.setPower(intake_motor);
        robot.dcMotor6.setPower(conveyor_motor);

//        robot.wgArmMovementHandler();


//        robot.wg_left.setPosition(wg_left);
//        robot.wg_right.setPosition(wg_right);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        }




}
