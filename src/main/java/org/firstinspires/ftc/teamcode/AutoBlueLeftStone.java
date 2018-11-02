package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Blue Left Stone", group="Autonomous")
//@Disabled
public class AutoBlueLeftStone extends LinearOpMode {

    /* Declare OpMode members. */
    TeleOpHardware1   robot           = new TeleOpHardware1();
    DriveByTimeBasics   Drive           = new DriveByTimeBasics();

    ElapsedTime StartTimer = new ElapsedTime();

    // Movement Functions
    public final void Forward (int time, double power)
    {
        //All motors forward at set power
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(power);

        //Wait For input time
        sleep(time);

        //Stop all motors after time
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    public final void Stop ()
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    //Diagonals not yet used
    public void DiagonalForwardLeft (int time, double power)
    {
        //All motors forward
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
        //Some Forward Some Back at set power
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(-power);

        //Wait For input time
        sleep(time);

        //Stop all motors after time
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
        //All motors backwards at set power
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(-power);

        //Wait For input time
        sleep(time);

        //Stop all motors after time
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
        //Some motors forward other back at set power
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);

        //Wait For input time
        sleep(time);

        //Stop all motors after time
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
        //Right wheels forward, left wheels back at set power
        robot.DriveLeftBack.setPower(-power);
        robot.DriveRightBack.setPower(power);
        robot.DriveRightFront.setPower(power);
        robot.DriveLeftFront.setPower(-power);

        //Wait For input time
        sleep(time);

        //Stop all motors after time
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    public final void TurnLeft (int time, double power)
    {

        //Left wheels forward, right wheels back at set power
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);

        //Wait For input time
        sleep(time);

        //Stop all motors after time
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower(0);
    }

    public final void GyroTurn (int degrees, double power)
    {
        //Define gyro angle
        float GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if(degrees <= -1)
        {
            while (GyroAngle >= degrees && opModeIsActive())
            {
                //While the gyro is reading degrees that is not the desired, turn right or left
                robot.DriveLeftBack.setPower(-power);
                robot.DriveRightBack.setPower(power);
                robot.DriveRightFront.setPower(power);
                robot.DriveLeftFront.setPower(-power);

                //Update gyro angle
                GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                telemetry.addData("Angle", GyroAngle);
                telemetry.update();
            }
        }
        else if (degrees >= 1)
            while (GyroAngle <= degrees && opModeIsActive())
            {
                //While the gyro is reading degrees that is not the desired, turn right or left
                robot.DriveLeftBack.setPower(-power);
                robot.DriveRightBack.setPower(power);
                robot.DriveRightFront.setPower(power);
                robot.DriveLeftFront.setPower(-power);

                //Update gyro angle
                GyroAngle = robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                telemetry.addData("Angle", GyroAngle);
                telemetry.update();
            }

        //Stop
        Stop();
    }

    public void runOpMode() {
        robot.init(hardwareMap);

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready To Run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.RelicElbow.setPosition(1);

        //Arm up
        robot.JewelArm.setPosition(0);

        //Close grippers, grab block
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

        //If red, because we are on blue, turn into the red and knock it off
        if (robot.JewelSensor.red() >= .5)
        {
            TurnLeft(250, .25);

            //Raise Arm
            robot.JewelArm.setPosition(0);
            sleep(1000);

            TurnRight(250, .25);
        }

        //If blue, because we are on blue, turn away from blue and knock red off
        else if (robot.JewelSensor.blue() >= .5)
        {
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

        }
        //Backward(1600, .25);

        //Turn off color sensor
        robot.JewelSensor.enableLed(false);

        //Approach Cipher

        Backward(1000, .4);

        GyroTurn(-65, .15);

        // Lower Lift
        robot.Lift.setPower(1);
        sleep(1500);
        robot.Lift.setPower(0);

        Forward(5000, .2);

        //Release Grippers
        robot.Intake.setPower(-1);
        sleep(500);
        robot.Intake.setPower(0);

        //Back away from glyph, stay in safe zone
        Backward(500, .1);
    }
}
