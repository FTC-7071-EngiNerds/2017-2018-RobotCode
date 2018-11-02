package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Blue Right Stone", group="Autonomous")
//@Disabled
public class AutoBlueRightStone extends LinearOpMode {

    /* Declare OpMode members. */
    TeleOpHardware1   robot           = new TeleOpHardware1();

    int TimerSeconds;

    ElapsedTime StartTimer = new ElapsedTime();

    ModernRoboticsI2cRangeSensor RightRange;
    ModernRoboticsI2cRangeSensor LeftRange;

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

    //Diagonals not yet used
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

    //Diagonals not yet used
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

    //Diagonals not yet used
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

    //Diagonals not yet used
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

    public final void GyroStrafeDistance (int Distance, double power)
    {

        double LeftRangeDistance = LeftRange.getDistance(DistanceUnit.CM);

        while (Distance >= LeftRangeDistance && opModeIsActive())
        {
            float GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            LeftRangeDistance = LeftRange.getDistance(DistanceUnit.CM);

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

            telemetry.addData("Angle",
                    GyroAngle);
            telemetry.addData("Distance", LeftRangeDistance);
            telemetry.update();

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

            TimerSeconds += 25;
            sleep(25);

        }

        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    public void runOpMode() {
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

        //Raise arm
        robot.JewelArm.setPosition(0);
        robot.RelicElbow.setPosition(1);

        //Pull in blocks
        robot.Intake.setPower(1);
        sleep (1000);
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

        //If reads red, because we are on blue, turn into the red and knock it off
        if (robot.JewelSensor.red() >= .5)
        {
            TurnLeft(250, .25);

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep(1000);

            TurnRight(250, .25);
        }

        //If reads blue, because we are on blue, turn away from blue and knock red off
        else if (robot.JewelSensor.blue() >= .5) {

            TurnRight(250, .25);

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep(1000);

            TurnLeft(250, .25);
        }

        else
        {

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep (1000);

            //Anything but red or blue do nothing
        }

        //Turn off color sensor
        robot.JewelSensor.enableLed(false);

        //Approach Cipher

        GyroStrafeTime(2500, .5);

        GyroStrafeDistance(41,.5);

        Backward(1000, .5);

        // Lower Lift
        robot.Lift.setPower(1);
        sleep(1500);
        robot.Lift.setPower(0);

        //Enter Cipher
        Forward(5000, .1);

        //Release grips
        robot.Intake.setPower(-1);
        sleep(500);

        //Back away from glyph, stay in safe zone
        Backward(500, .1);

        robot.Intake.setPower(0);
    }
}
