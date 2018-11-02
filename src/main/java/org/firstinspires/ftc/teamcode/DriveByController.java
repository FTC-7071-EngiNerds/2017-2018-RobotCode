package org.firstinspires.ftc.teamcode;

/**
 * Created by Jared Morgan, Team Enginerds on 10/20/2017.
 */

public class DriveByController
{

    TeleOpHardware1 robot           = new TeleOpHardware1();              // Use custom hardware

    /* Constructor */
    public DriveByController() {
    }

    public void Forward (double power)
    {
        robot.DriveLeftBack.setPower(power/4);
        robot.DriveRightBack.setPower(power/4);
        robot.DriveRightFront.setPower(power/4);
        robot.DriveLeftFront.setPower(power/4);
    }

    public void DiagonalForwardLeft (double power1, double power2)
    {
        robot.DriveLeftBack.setPower((-power1 + power2)/6);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower((-power1 + power2)/6);
        robot.DriveLeftFront.setPower(0);
    }

    public void Left (double power)
    {
        robot.DriveLeftBack.setPower(-power/4);
        robot.DriveRightBack.setPower(power/4);
        robot.DriveRightFront.setPower(-power/4);
        robot.DriveLeftFront.setPower(power/4);
    }

    public void DiagonalBackLeft (double power1, double power2)
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower((power1 + power2)/6);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower((power1 + power2)/6);
    }

    public void Backward (double power)
    {
        robot.DriveLeftBack.setPower(-power/4);
        robot.DriveRightBack.setPower(-power/4);
        robot.DriveRightFront.setPower(-power/4);
        robot.DriveLeftFront.setPower(-power/4);
    }

    public void DiagonalBackRight (double power1, double power2)
    {
        robot.DriveLeftBack.setPower((-power1 + power2)/6);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower((-power1 + power2)/6);
        robot.DriveLeftFront.setPower(0);
    }

    public void Right (double power)
    {
        robot.DriveLeftBack.setPower(power/4);
        robot.DriveRightBack.setPower(-power/4);
        robot.DriveRightFront.setPower(power/4);
        robot.DriveLeftFront.setPower(-power/4);
    }

    public void DiagonalFowardRight (double power1, double power2)
    {
        robot.DriveLeftBack.setPower(0);
        robot.DriveRightBack.setPower((power1 + power2)/6);
        robot.DriveRightFront.setPower(0);
        robot.DriveLeftFront.setPower((power1 + power2)/6);
    }

    public void Turn (double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power);
    }

    public void HardRightTurn (double power)
    {
        robot.DriveLeftBack.setPower(power);
        robot.DriveRightBack.setPower(-power/10);
        robot.DriveRightFront.setPower(-power/10);
        robot.DriveLeftFront.setPower(power);
    }

    public void HardLeftTurn (double power)
    {
        robot.DriveLeftBack.setPower(power/10);
        robot.DriveRightBack.setPower(-power);
        robot.DriveRightFront.setPower(-power);
        robot.DriveLeftFront.setPower(power/10);
    }
}
