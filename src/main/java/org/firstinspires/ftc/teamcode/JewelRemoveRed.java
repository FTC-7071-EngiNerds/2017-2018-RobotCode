package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Remove Jewel Red", group="Autonomous")
//@Disabled
public class JewelRemoveRed extends LinearOpMode {

    /* Declare OpMode members. */
    TeleOpHardware1   robot           = new TeleOpHardware1();
    DriveByTimeBasics   Drive           = new DriveByTimeBasics();

    // Movement Functions
    public final void Forward (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public void DiagonalForwardLeft (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(0);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public final void Left (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public void DiagonalBackLeft (int time, double power)
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public final void Backward (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public void DiagonalBackRight (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(0);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public final void Right (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public void DiagonalForwardRight (int time, double power)
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public final void TurnRight (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public final void TurnLeft (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready To Run");    //
        telemetry.update();
        //robot.Gripper.setPosition(.5);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.JewelArm.setPosition(.99);

        robot.Intake.setPower(1);
        sleep (1000);
        robot.Intake.setPower(0);

        //Raise Lift
        robot.Lift.setPower(-1);
        sleep(1000);
        robot.Lift.setPower(0);

        //Drop Arm
        robot.JewelArm.setPosition(.35);
        sleep(1000);

        //Sense and React to color
        robot.JewelSensor.enableLed(true);
        if (robot.JewelSensor.red() >= .5)
        {
            TurnRight(150, .25);

            //Raise Arm
            robot.JewelArm.setPosition(.99);
            sleep(1000);

            TurnLeft(150, .25);
        }
        else if (robot.JewelSensor.blue() >= .5)
        {
            TurnLeft(150, .25);

            //Raise Arm
            robot.JewelArm.setPosition(.99);
            sleep(1000);

            TurnRight(150, .25);
        }
        else
        {

            //Raise Arm
            robot.JewelArm.setPosition(.99);
            sleep (1000);

        }

        robot.JewelSensor.enableLed(false);

        // Lower Lift
        robot.Lift.setPower(1);
        sleep(1000);
        robot.Lift.setPower(0);
    }
}
