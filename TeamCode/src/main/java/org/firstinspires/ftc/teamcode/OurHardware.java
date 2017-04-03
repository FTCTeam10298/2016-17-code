package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class OurHardware
{
    /* Public OpMode members. */
    public DcMotor  leftMotorF      = null;
    public DcMotor  leftMotorB      = null;
    public DcMotor  rightMotorF     = null;
    public DcMotor  rightMotorB     = null;
    public DcMotor  launchingMotor  = null;
    public DcMotor loaderMotor      = null;

    public Servo    beaconpusherR    = null;
    public Servo    beaconpusherL    = null;

    /* local OpMode members. */
    HardwareMap hwMap               = null;
    private ElapsedTime period      = new ElapsedTime();

    /* Constructor */
    public OurHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        leftMotorF      = hwMap.dcMotor.get("left_drive_front");
        leftMotorB      = hwMap.dcMotor.get("left_drive_back");
        rightMotorF     = hwMap.dcMotor.get("right_drive_front");
        rightMotorB     = hwMap.dcMotor.get("right_drive_back");
        launchingMotor  = hwMap.dcMotor.get("launching_motor");
        loaderMotor     = hwMap.dcMotor.get("arm_motor");

        // Set direction for all motors
        leftMotorF.setDirection(DcMotor.Direction.FORWARD);
        leftMotorB.setDirection(DcMotor.Direction.FORWARD);
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);
        launchingMotor.setDirection(DcMotor.Direction.REVERSE);
        loaderMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        launchingMotor.setPower(0);
        loaderMotor.setPower(0);

        // Set (almost) all motors to run with encoders.
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // This motor does not use an encoder
        loaderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize all installed servos.
        beaconpusherR = hwMap.servo.get("right_beacon");
        beaconpusherL = hwMap.servo.get("left_beacon");
        beaconpusherR.setPosition(0.1);
        beaconpusherL.setPosition(0.9);
    }

    /*
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick (long periodMs) {

        long remaining = periodMs - (long)period.milliseconds();

        // Sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

