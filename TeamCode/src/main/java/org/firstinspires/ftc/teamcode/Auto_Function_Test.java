///*
//Copyright (c) 2016 Robert Atkinson
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Robert Atkinson nor the names of his contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Color;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
///**
// * This file illustrates the concept of driving a path based on time.
// * It uses the common Pushbot hardware class to define the drive on the robot.
// * The code is structured as a LinearOpMode
// *
// * The code assumes that you do NOT have encoders on the wheels,
// *   otherwise you would use: PushbotAutoDriveByEncoder;
// *
// *   The desired path in this example is:
// *   - Drive forward for 3 seconds
// *   - Spin right for 1.3 seconds
// *   - Drive Backwards for 1 Second
// *   - Stop and close the claw.
// *
// *  The code is written in a simple form with no optimizations.
// *  However, there are several ways that this type of sequence could be streamlined,
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@Autonomous(name="Auto Function Test", group="Pushbot")
//@Disabled
//public class Auto_Function_Test extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    OurHardware         robot   = new OurHardware();   // Use a Pushbot's hardware
//    private ElapsedTime     runtime = new ElapsedTime();
//    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device
//    ModernRoboticsI2cColorSensor color = null;
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Neverrest 40 Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    // These constants define the desired driving/control characteristics
//    // The can/should be tweaked to suite the specific robot drive train.
//    static final double     DRIVE_SPEED             = 0.25;     // Nominal speed for better accuracy.
//    static final double     TURN_SPEED              = 0.25;     // Nominal half speed for better accuracy.
//
//    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
//    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
//
//
//    static final double     FORWARD_SPEED = 0.6;
//
//    @Override
//    public void runOpMode() {
//
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//        // Send telemetry message to alert driver that we are calibrating;
//        telemetry.addData("Status", "Calibrating Gyro");    //
//        telemetry.update();
//
//        // Init Gyro ------------------------------------------------------------------------------------
//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
//        gyro.calibrate();
//
//        // make sure the gyro is calibrated before continuing
//        while (gyro.isCalibrating())  {
//            sleep(50);
//            idle();
//        }
//        gyro.resetZAxisIntegrator();
//
//        // Init Color Sensor
//        color = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("color");
//
//        // Set the LED in the beginning
//        color.enableLed(false);
//
//        // claw init
//        robot.claw.setPosition(.3);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Ready to run");    //
//        telemetry.update();
//
//        robot.init(hardwareMap);
//        robot.loaderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.loaderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.loaderMotor.setTargetPosition(0);
//        robot.loaderMotor.setPower(0.0);
//
//        robot.init(hardwareMap);
//        robot.launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.launchingMotor.setTargetPosition(0);
//        robot.launchingMotor.setPower(0.0);
////        robot.loaderMotor.setMaxSpeed(1000);
////        robot.loaderMotor.setMaxSpeed(1000);
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//// TEST --------------------------------------------------------------------------------------------
//
////      Drive straight by inches using given power
////          RobotDrivePosition(24, .25);
//
////      Turn by degree using given power
////          RobotTurn(270, .25);
//
////        Drive sideways by inches using given power
////            RobotSidewaysDrive(3000);
//
////      Drive Sideways for time (power either 1 or negative 1
////        SidewaysDriveTime(2000, 1);
////        sleep (5000);
////        SidewaysDriveTime(2000, -1);
////        sleep (5000);
////        RobotSidewaysDrive(4000);
//
////        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
////        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
////        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
////        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
////        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
////        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
////        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
////        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
////        gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second
//
//        // convert the RGB values to HSV values.
//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F,0F,0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
// //       while(opModeIsActive()) {
// //           Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
////
//  //          // send the info back to driver station using telemetry function.
//    //        telemetry.addData("Clear", color.alpha());
//      //      telemetry.addData("Red  ", color.red());
////            telemetry.addData("Green", color.green());
//  ///          telemetry.addData("Blue ", color.blue());
//       //     telemetry.addData("Hue", hsvValues[0]);
//     //       telemetry.update();
//    //    }
//DriveUntilBlue(.5);
//
////--------------------------------------------------------------------------------------------------
//
// /*       // Drive forward for 3 seconds
//        robot.leftMotorF.setPower(0.2);
//        robot.leftMotorB.setPower(0.2);
//        robot.rightMotorF.setPower(0.2);
//        robot.rightMotorB.setPower(0.2);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        robot.leftMotorF.setPower(0.0);
//        robot.leftMotorB.setPower(0.0);
//        robot.rightMotorF.setPower(0.0);
//        robot.rightMotorB.setPower(0.0);
//
//        sleep(1000);
//   */
//    }
//
//    // FUNCTIONS -----------------------------------------------------------------------------------
//
//
//    void BallLaunch(int ballsToLaunch)
//    {
//        for (int ballsLaunched = 0; ballsLaunched < ballsToLaunch; ballsLaunched++) {
//            robot.loaderMotor.setPower(.1);
//            sleep(1000);
//            robot.loaderMotor.setPower(0);
//
//            robot.launchingMotor.setTargetPosition(3350 * ballsLaunched);
//            robot.launchingMotor.setPower(.5);
//            while (robot.launchingMotor.isBusy()) {
//                telemetry.addData("encoder", "%d", robot.launchingMotor.getCurrentPosition());
//                telemetry.update();
//            }
//            robot.launchingMotor.setPower(0.0);
//
//            robot.loaderMotor.setPower(-.1);
//            sleep(1100);
//            robot.loaderMotor.setPower(0);
//
//            sleep(500);
//            robot.claw.setPosition(0);
//            sleep(500);
//
//            robot.loaderMotor.setPower(.1);
//            sleep(1000);
//            robot.claw.setPosition(0.15);
//            sleep(100);
//
//            robot.loaderMotor.setPower(-.2);
//            sleep(700);
//            robot.loaderMotor.setPower(.2);
//            sleep(700);
//            robot.loaderMotor.setPower(0);
//            sleep(500);
//
//            /*
//            robot.launchingMotor.setTargetPosition(3350 * 2);
//            robot.launchingMotor.setPower(.5);
//            while (robot.launchingMotor.isBusy()) {
//                telemetry.addData("encoder", "%d", robot.launchingMotor.getCurrentPosition());
//                telemetry.update();
//            }
//            robot.launchingMotor.setPower(0.0);
//            sleep(1000);
//            */
//        }
//    }
//
//
//    void RobotDriveTime(int time, double power)
//    {
//        DrivePowerAll(-power);
//
//        sleep(time);
//
//        DrivePowerAll(0);
//
//    }
//// RobotDrivePosition (works best with .25 power)
//    void RobotDrivePosition(int inches, double power)
//    {
//        int position = -inches*90 ;
//
//        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        DrivePowerAll(-power);
//
//        robot.leftMotorF.setTargetPosition(position);
//        robot.rightMotorF.setTargetPosition(position);
//        robot.rightMotorB.setTargetPosition(position);
//        robot.leftMotorB.setTargetPosition(position);
//
//        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.leftMotorF.isBusy() || robot.rightMotorF.isBusy() || robot.rightMotorB.isBusy() || robot.leftMotorB.isBusy()) {
//            telemetry.addData("encoder", "%d", robot.leftMotorF.getCurrentPosition());
//            telemetry.update();
//        }
//
//        DrivePowerAll(0);
//
//    }
////RobotTurn
//    void RobotTurn (int degree, double power)
//    {
//        int position = degree*19;
//
//        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.leftMotorF.setPower(-power);
//        robot.rightMotorF.setPower(power);
//        robot.rightMotorB.setPower(power);
//        robot.leftMotorB.setPower(-power);
//
//        robot.leftMotorF.setTargetPosition(-position);
//        robot.rightMotorF.setTargetPosition(position);
//        robot.rightMotorB.setTargetPosition(position);
//        robot.leftMotorB.setTargetPosition(-position);
//
//        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.leftMotorF.isBusy() || robot.rightMotorF.isBusy() || robot.rightMotorB.isBusy() || robot.leftMotorB.isBusy()) {
//            telemetry.addData("encoder", "%d", robot.leftMotorF.getCurrentPosition());
//            telemetry.update();
//        }
//
//        DrivePowerAll(0);
//
//    }
//
//    void RobotSidewaysDrive (int inches)
//    {
//
//        int position = inches;
//
//        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.rightMotorF.setPower(-1);
//        robot.rightMotorB.setPower(1);
//        robot.leftMotorB.setPower(-1);
//        robot.leftMotorF.setPower(1);
//
//        robot.rightMotorF.setTargetPosition(-position);
//        robot.rightMotorB.setTargetPosition(position);
//        robot.leftMotorB.setTargetPosition(-position);
//        robot.leftMotorF.setTargetPosition(position);
//
//        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.rightMotorB.isBusy() || robot.leftMotorF.isBusy() || robot.leftMotorB.isBusy() || robot.rightMotorF.isBusy()) {
//            telemetry.addData("encoder", "%d", robot.leftMotorF.getCurrentPosition());
//            telemetry.update();
//        }
//
//        DrivePowerAll(0);
//    }
//
//    void DrivePowerAll (double power)
//    {
//        robot.leftMotorF.setPower(power);
//        robot.rightMotorF.setPower(power);
//        robot.rightMotorB.setPower(power);
//        robot.leftMotorB.setPower(power);
//    }
//
//    void SidewaysDriveTime (int time, double power)
//    {
//        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        if(power > 0) // right
//        {
//            robot.leftMotorF.setPower(-power);
//            robot.leftMotorB.setPower(power);
//            robot.rightMotorB.setPower(-power);
//            robot.rightMotorF.setPower(power);
//        }
//        else // left
//        {
//            robot.rightMotorF.setPower(power);
//            robot.rightMotorB.setPower(-power);
//            robot.leftMotorB.setPower(power);
//            robot.leftMotorF.setPower(-power);
//        }
//
//
//        sleep(time);
//
//        DrivePowerAll(0);
//    }
//
//    void DriveUntilBlue (double power)
//    {
//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F,0F,0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
//        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
//
//          // send the info back to driver station using telemetry function.
//            telemetry.addData("Clear", color.alpha());
//            telemetry.addData("Red  ", color.red());
//            telemetry.addData("Green", color.green());
//            telemetry.addData("Blue ", color.blue());
//            telemetry.addData("Hue", hsvValues[0]);
//            telemetry.update();
//        Boolean colorFound = false;
//
//        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (!colorFound)
//        {
//            DrivePowerAll(power);
//            if (color.blue()>0)
//            {
//                colorFound = true;
//            }
//
//        }
//        if (colorFound)
//            DrivePowerAll(0);
//    }
//
//// -------GYRO------------------------------------------------------------------------------------------
//    /**
//     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Driver stops the opmode running.
//     *
//     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
//     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     */
//    public void gyroDrive ( double speed,
//                            double distance,
//                            double angle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int)(-distance * COUNTS_PER_INCH);
//            newLeftTarget = robot.leftMotorF.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.rightMotorF.getCurrentPosition() + moveCounts;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            robot.leftMotorF.setTargetPosition(newLeftTarget);
//            robot.rightMotorF.setTargetPosition(newRightTarget);
//            robot.leftMotorB.setTargetPosition(newLeftTarget);
//            robot.rightMotorB.setTargetPosition(newRightTarget);
//
//            robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = -speed;
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.leftMotorF.setPower(speed);
//            robot.rightMotorF.setPower(speed);
//            robot.leftMotorB.setPower(speed);
//            robot.rightMotorB.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (robot.leftMotorF.isBusy() && robot.rightMotorF.isBusy() && robot.rightMotorB.isBusy() && robot.rightMotorB.isBusy())) {
//
//                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if any one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.leftMotorF.setPower(leftSpeed);
//                robot.rightMotorF.setPower(rightSpeed);
//                robot.leftMotorB.setPower(leftSpeed);
//                robot.rightMotorB.setPower(rightSpeed);
//
//                // Display drive status for the driver.
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotorF.getCurrentPosition(),
//                        robot.rightMotorF.getCurrentPosition());
//                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            DrivePowerAll(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    /**
//     *  Method to spin on central axis to point in a new direction.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the heading (angle)
//     *  2) Driver stops the opmode running.
//     *
//     * @param speed Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     */
//    public void gyroTurn (  double speed, double angle) {
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(-speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//    }
//
//    /**
//     *  Method to obtain & hold a heading for a finite amount of time
//     *  Move will stop once the requested time has elapsed
//     *
//     * @param speed      Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     * @param holdTime   Length of time (in seconds) to hold the specified heading.
//     */
//    public void gyroHold( double speed, double angle, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            // Update telemetry & Allow time for other processes to run.
//            onHeading(speed, angle, P_TURN_COEFF);
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        DrivePowerAll(0);
//    }
//
//    /**
//     * Perform one cycle of closed loop heading control.
//     *
//     * @param speed     Desired speed of turn.
//     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
//     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                  If a relative angle is required, add/subtract from current heading.
//     * @param PCoeff    Proportional Gain coefficient
//     * @return
//     */
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//        robot.leftMotorF.setPower(leftSpeed);
//        robot.rightMotorF.setPower(rightSpeed);
//        robot.leftMotorB.setPower(leftSpeed);
//        robot.rightMotorB.setPower(rightSpeed);
//
//        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//
//    /**
//     * getError determines the error between the target angle and the robot's current heading
//     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
//     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
//     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
//     */
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - gyro.getIntegratedZValue();
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//    /**
//     * returns desired steering force.  +/- 1 range.  +ve = steer left
//     * @param error   Error angle in robot relative degrees
//     * @param PCoeff  Proportional Gain Coefficient
//     * @return
//     */
//    public double getSteer(double error, double PCoeff) {
//        return Range.clip(error * PCoeff, -1, 1);
//    }
////-------------------------------------------------------------------------------------------------
//}