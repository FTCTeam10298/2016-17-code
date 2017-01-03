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
    public DcMotor  armMotor        = null;

    public Servo    claw            = null;
    public Servo    beaconpusher    = null;

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

        // Define and Initialize Motors
        leftMotorF      = hwMap.dcMotor.get("left_drive_front");
        leftMotorB      = hwMap.dcMotor.get("left_drive_back");
        rightMotorF     = hwMap.dcMotor.get("right_drive_front");
        rightMotorB     = hwMap.dcMotor.get("right_drive_back");

        launchingMotor  = hwMap.dcMotor.get("launching_motor");
        armMotor        = hwMap.dcMotor.get("arm_motor");


        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);

        launchingMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

        launchingMotor.setPower(0);
        armMotor.setPower(0);

        // Set all motors to run with encoders.
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize all installed servos.
        claw = hwMap.servo.get("claw");
        claw.setPosition(0.24);

        beaconpusher = hwMap.servo.get("beacon");
        beaconpusher.setPosition(0.0);
    }

    /*
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
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

