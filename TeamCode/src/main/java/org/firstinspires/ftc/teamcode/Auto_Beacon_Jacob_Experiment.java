/*
Copyright (c) 2016 Robert Atkinson (original code for Pushbot), FTC team #10298 Brain Stormz (making it fit our bot)

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 *  Autonomous program for Blue alliance
 */

@Autonomous(name="Auto Blue with Beacon", group="Ourbot")
//@Disabled
public class Auto_Beacon_Jacob_Experiment extends LinearVisionOpMode {

    /* Declare OpMode members. */
            OurHardware     robot   = new OurHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        waitForVisionStart();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        // claw init
        robot.claw.setPosition(.3);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        robot.init(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setPower(0.0);
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.init(hardwareMap);
        robot.launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.launchingMotor.setTargetPosition(0);
        robot.launchingMotor.setPower(0.0);
//        robot.armMotor.setMaxSpeed(1000);
//        robot.armMotor.setMaxSpeed(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //Nutn but net

        /*
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(.1);
        sleep(1000);
        robot.armMotor.setPower(0);

        robot.launchingMotor.setTargetPosition(3350);
        robot.launchingMotor.setPower(.5);
        while (robot.launchingMotor.isBusy()) {
            telemetry.addData("encoder", "%d", robot.launchingMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.launchingMotor.setPower(0.0);

        robot.armMotor.setPower(-.1);
        sleep(1100);
        robot.armMotor.setPower(0);

        sleep(500);
        robot.claw.setPosition(0);
        sleep(500);

        robot.armMotor.setPower(.1);
        sleep(1000);
        robot.claw.setPosition(0.15);
        sleep(100);

        robot.armMotor.setPower(-.2);
        sleep(700);
        robot.armMotor.setPower(.2);
        sleep(700);
        robot.armMotor.setPower(0);
        sleep(500);

        robot.launchingMotor.setTargetPosition(3350*2);
        robot.launchingMotor.setPower(.5);
        while (robot.launchingMotor.isBusy()) {
            telemetry.addData("encoder", "%d", robot.launchingMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.launchingMotor.setPower(0.0);
        sleep(1000);

        robot.leftMotorF.setPower(.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(.5);
        robot.rightMotorB.setPower(.5);

        sleep(1300);

        robot.leftMotorF.setPower(-.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(-.5);
        robot.rightMotorB.setPower(.5);

        sleep(1100);

        robot.leftMotorF.setPower(.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(.5);
        robot.rightMotorB.setPower(.5);

        sleep(2500);

        robot.leftMotorF.setPower(0.0);
        robot.rightMotorF.setPower(0.0);
        robot.leftMotorB.setPower(0.0);
        robot.rightMotorB.setPower(0.0);
        */

        //forward
        robot.leftMotorF.setPower(.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(.5);
        robot.rightMotorB.setPower(.5);

        sleep(1000);

        //left
        robot.leftMotorF.setPower(-.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(-.5);
        robot.rightMotorB.setPower(.5);

        sleep(750);

        //forward
        robot.leftMotorF.setPower(.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(.5);
        robot.rightMotorB.setPower(.5);

        sleep(500);

        robot.leftMotorF.setPower(-.5);
        robot.rightMotorF.setPower(.5);
        robot.leftMotorB.setPower(-.5);
        robot.rightMotorB.setPower(.5);

        sleep(750);

        //while beacon !inCenter or confidenceRating > 75% {drive forward}
        while (beacon.getAnalysis().getCenter().x < 400 || beacon.getAnalysis().getConfidence() > 75) {
            robot.leftMotorF.setPower(.5);
            robot.leftMotorB.setPower(.5);
            robot.rightMotorF.setPower(.5);
            robot.rightMotorB.setPower(.5);
            sleep(10);
        }
        if (beacon.getAnalysis().getColorString() == "blue, red") {
            // Move left
            robot.leftMotorF.setPower(.5);
            robot.leftMotorB.setPower(-.5);
            robot.rightMotorF.setPower(-.5);
            robot.rightMotorB.setPower(.5);
        }
        if (beacon.getAnalysis().getColorString() == "red, blue") {
            // Move right
            robot.leftMotorF.setPower(-.5);
            robot.leftMotorB.setPower(.5);
            robot.rightMotorF.setPower(.5);
            robot.rightMotorB.setPower(-.5);
        }
        sleep(250);
    }
}
