package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Timer;

@Autonomous(name="Red Left Stone", group="Autonomous")
//@Disabled
public class AutoRedLeftStone extends LinearOpMode {

    /* Declare OpMode members. */
    TeleOpHardware1   robot           = new TeleOpHardware1();

    ModernRoboticsI2cRangeSensor RightRange;
    ModernRoboticsI2cRangeSensor LeftRange;


    //VuforiaLocalizer vuforia;

    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


    int TimerSeconds;

    ElapsedTime StartTimer = new ElapsedTime();

    // Movement Functions
    public final void Stop ()
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }
    public final void Forward (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        Stop();
    }
    public void DiagonalForwardLeft (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(0);
        sleep(time);
        Stop();
    }
    public final void Left (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        Stop();
    }
    public void DiagonalBackLeft (int time, double power)
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        Stop();
    }
    public final void Backward (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        Stop();
    }
    public void DiagonalBackRight (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(0);
        sleep(time);
        Stop();
    }
    public final void Right (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        Stop();
    }
    public void DiagonalForwardRight (int time, double power)
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        Stop();
    }
    public final void TurnRight (int time, double power)
    {
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(-power);
        sleep(time);
        Stop();
    }
    public final void TurnLeft (int time, double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);
        sleep(time);
        Stop();
    }

    //Strafe using gyro to keep robot straight
    //Positive power = right, negative power = left
    public final void GyroStrafeDistance (int Distance, double power)
    {
        //Define variable for the right range senson that measures centimeters
        double RightRangeDistance = RightRange.getDistance(DistanceUnit.CM);

        //While the robot's distance is less than the wanted distance do the following
        while (Distance >= RightRangeDistance && opModeIsActive())
        {
            //Define variable for gyro angle
            float GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            //Update the variable for the range sensor
            RightRangeDistance = RightRange.getDistance(DistanceUnit.CM);

            //If the gyro's read angle is more than 3 degrees turn right until it is not
            if (GyroAngle > 3 )
            {
                while(GyroAngle > 3 && opModeIsActive())
                {
                    //Turn Right
                    robot.DriveLeftBack.setPower(-.1);
                    robot.DriveRightBack.setPower(.1);
                    robot.DriveRightFront.setPower(.1);
                    robot.DriveLeftFront.setPower(-.1);

                    //Update the gyro's angle
                    GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }

            //If the gyro's read angle is less than -3 degrees turn left until it is not
            else if (GyroAngle < -3)
            {
                while (GyroAngle < -3 && opModeIsActive())
                {
                    //Turn Left
                    robot.DriveLeftBack.setPower(.1);
                    robot.DriveRightBack.setPower(-.1);
                    robot.DriveRightFront.setPower(-.1);
                    robot.DriveLeftFront.setPower(.1);

                    //Update the gyro's angle
                    GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }

            //Strafe (right or left)
            robot.DriveLeftBack.setPower(-power);
            robot.DriveRightBack.setPower(power);
            robot.DriveRightFront.setPower(-power);
            robot.DriveLeftFront.setPower(power);

            //Send feedback to phone
            telemetry.addData("Angle",
                    GyroAngle);
            telemetry.addData("Distance", RightRangeDistance);
            telemetry.update();

            //End of loop, robot will repeat the above until it reaches the wanted distance
        }
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    public final void GyroStrafeTime (int time, double power)
    {
        TimerSeconds = 0;

        while (time >= TimerSeconds && opModeIsActive())
        {
            float GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


            if (GyroAngle > 4 )
            {
                while(GyroAngle > 4 && opModeIsActive())
                {
                    //Turn Right
                    robot.DriveLeftBack.setPower(-.25);
                    robot.DriveRightBack.setPower(.25);
                    robot.DriveRightFront.setPower(.25);
                    robot.DriveLeftFront.setPower(-.25);

                    GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }

            else if (GyroAngle < -4)
            {
                while (GyroAngle < -4 && opModeIsActive())
                {
                    //Turn Left
                    robot.DriveLeftBack.setPower(.25);
                    robot.DriveRightBack.setPower(-.25);
                    robot.DriveRightFront.setPower(-.25);
                    robot.DriveLeftFront.setPower(.25);

                    GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }

            robot.DriveLeftBack.setPower(-power);
            robot.DriveRightBack.setPower(power);
            robot.DriveRightFront.setPower(-power);
            robot.DriveLeftFront.setPower(power);

            telemetry.addData("zAngle",
                    GyroAngle);
            telemetry.addData("Timer", TimerSeconds);
            telemetry.update();

            TimerSeconds += 25;
            sleep(25);

        }
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    // Postive degrees and power for right, negative degrees and angle for left
    public final void GyroTurn (int degrees, double power)
    {
        //Define gyro angle
        float GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //While the gyro is reading degrees that is not the desired, turn right or left
        while (GyroAngle != degrees)
        {
            //
            robot.DriveLeftBack.setPower(-.1);
            robot.DriveRightBack.setPower(.1);
            robot.DriveRightFront.setPower(.1);
            robot.DriveLeftFront.setPower(-.1);

            //Update gyro angle
            GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }

        //Stop
        Stop();
    }

    /**public final void GyroStrafeVuforia (int time, double power)
    {
        TimerSeconds = 0;

        while (time >= TimerSeconds && opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN)
        {
            float GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (GyroAngle > 4 )
            {
                while(GyroAngle > 4)
                {
                    //Turn Right
                    robot.DriveLeftBack.setPower(-.25);
                    robot.DriveRightBack.setPower(.25);
                    robot.DriveRightFront.setPower(.25);
                    robot.DriveLeftFront.setPower(-.25);

                    GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }

            else if (GyroAngle < -4)
            {
                while (GyroAngle < -4)
                {
                    //Turn Left
                    robot.DriveLeftBack.setPower(.25);
                    robot.DriveRightBack.setPower(-.25);
                    robot.DriveRightFront.setPower(-.25);
                    robot.DriveLeftFront.setPower(.25);

                    GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }

            robot.DriveLeftBack.setPower(-power);
            robot.DriveRightBack.setPower(power);
            robot.DriveRightFront.setPower(-power);
            robot.DriveLeftFront.setPower(power);

            telemetry.addData("zAngle",
                    GyroAngle);
            telemetry.addData("Timer", TimerSeconds);
            telemetry.update();

            TimerSeconds += 10;
            sleep(10);

        }
    }
**/


    public void runOpMode()
    {
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        robot.init(hardwareMap);

        RightRange      = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"RightRange");
        LeftRange       = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"LeftRange");

        RightRange.setI2cAddress(I2cAddr.create8bit(0x3e));
        LeftRange.setI2cAddress(I2cAddr.create8bit(0x4e));

        //Gyro start borrowed from MRGyroSensor example
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        robot.ModernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        StartTimer.reset();
        while (!isStopRequested() && robot.ModernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(StartTimer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        float zAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        int rawZ = robot.ModernRoboticsI2cGyro.rawZ();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready To Run");
        telemetry.addData("Gyro Angle" , zAngle);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.RelicElbow.setPosition(1);
        //Raise jewel arm
        robot.JewelArm.setPosition(0);

        robot.Intake.setPower(1);
        sleep(1000);
        robot.Intake.setPower(0);

        //Raise Lift
        robot.Lift.setPower(-1);
        sleep(3000);
        robot.Lift.setPower(0);

        //Drop Arm
        robot.JewelArm.setPosition(.99);

        //Sense and React to color
        robot.JewelSensor.enableLed(true);
        sleep(2000);
        telemetry.addData("Red  ", robot.JewelSensor.red());
        telemetry.addData("Blue ", robot.JewelSensor.blue());
        telemetry.update();

        if(robot.JewelSensor.red() >= .1)
        {
            TurnRight(250, .25);

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep (1000);

            TurnLeft(250, .25);
        }
        else if (robot.JewelSensor.blue() >= .1)
        {

            TurnLeft(250, .25);

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep (1000);

            TurnRight(250, .25);
        }
        else
        {

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep (1000);

        }

        robot.JewelSensor.enableLed(false);

        //Approach Cipher

        GyroStrafeTime(2500, -.5);

        GyroStrafeDistance(41,.5);

        // Lower Lift
        robot.Lift.setPower(1);
        sleep(1500);
        robot.Lift.setPower(0);

        //Enter Cipher
        Forward(5000,.1);

        //Release Grips
        robot.Intake.setPower(-1);
        sleep(500);

        //Back away from glyph, stay in safe zone
        Backward(500,.1);

        robot.Intake.setPower(0);
    }
}