package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "DriveLeftFront"
 * Motor channel:  Right drive motor:        "DriveRightFront"
 * Servo channel:
 * Servo channel:
 */
public class TeleOpHardware1
{
    /* Public OpMode members. */
    //Motors
    public DcMotor  DriveLeftBack    = null;
    public DcMotor  DriveRightBack   = null;
    public DcMotor  DriveLeftFront   = null;
    public DcMotor  DriveRightFront  = null;
    public DcMotor  Lift             = null;
    public DcMotor  RelicExtender   = null;
    public DcMotor  Intake          = null;

    //Servos
    public Servo JewelArm   = null;
    public Servo RelicGrip  = null;
    public Servo RelicElbow   = null;

    //Sensors
    public ColorSensor JewelSensor;
    IntegratingGyroscope Gyro;
    ModernRoboticsI2cGyro ModernRoboticsI2cGyro;
    ModernRoboticsI2cRangeSensor RightRange;
    ModernRoboticsI2cRangeSensor LeftRange;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TeleOpHardware1() {
    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        DriveLeftFront  = hwMap.dcMotor.get("DriveLeftFront");
        DriveRightFront = hwMap.dcMotor.get("DriveRightFront");
        DriveLeftBack   = hwMap.dcMotor.get("DriveLeftBack");
        DriveRightBack  = hwMap.dcMotor.get("DriveRightBack");
        Lift            = hwMap.dcMotor.get("Lift");
        RelicExtender   = hwMap.dcMotor.get("RelicExtender");
        Intake          = hwMap.dcMotor.get("Intake");

        //Define and Initialize Servos
        JewelArm        = hwMap.servo.get("JewelArm");
        RelicGrip       = hwMap.servo.get("RelicGrip");
        RelicElbow      = hwMap.servo.get("RelicElbow");

        //Define and Initialize Sensors
        JewelSensor     = hwMap.colorSensor.get("JewelSensor");
        RightRange      = hwMap.get(ModernRoboticsI2cRangeSensor.class,"RightRange");
        LeftRange       = hwMap.get(ModernRoboticsI2cRangeSensor.class,"LeftRange");
        ModernRoboticsI2cGyro   = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        Gyro = (IntegratingGyroscope)ModernRoboticsI2cGyro;



        //Set Drive Directions
        //DriveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        DriveRightFront.setDirection(DcMotor.Direction.REVERSE);
        //DriveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        DriveRightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        DriveRightFront.setPower(0);
        DriveLeftFront.setPower(0);
        DriveRightBack.setPower(0);
        DriveLeftBack.setPower(0);
        Lift.setPower(0);

        //Define Run Modes
        DriveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        JewelSensor.setI2cAddress(I2cAddr.create7bit(0x2e));
        RightRange.setI2cAddress(I2cAddr.create8bit(0x3e));
        LeftRange.setI2cAddress(I2cAddr.create8bit(0x4e));
    }

    /***
     *
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
