package org.firstinspires.ftc.teamcode;

/**
 * Created by Jared Morgan, Team Enginerds on 10/15/2017.
 */

public class DriveByTimeBasics
{

    TeleOpHardware1 robot           = new TeleOpHardware1();              // Use custom hardware

    /* Constructor */
    public DriveByTimeBasics() {
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public final void Forward (int time)
    {
        robot.DriveLeftBack.setPower(.1);
        robot.DriveRightBack.setPower(.1);
        robot.DriveRightFront.setPower(.1);
        robot.DriveLeftFront.setPower(.1);
        sleep(time);
    }

    /*public void DiagonalForwardLeft (int time)
    {
        robot.DriveLeftBack.setPower(.08);
        robot.DriveRightBack.setPower(0);
        robot.DriveRightFront.setPower(.08);
        robot.DriveLeftFront.setPower(0);
        sleep(time);
    }
    */

    public final void Left (int time)
    {
        robot.DriveLeftBack.setPower(-.1);
        robot.DriveRightBack.setPower(.1);
        robot.DriveRightFront.setPower(-.1);
        robot.DriveLeftFront.setPower(.1);
        sleep(time);
    }

    public final void Backward (int time)
    {
        robot.DriveLeftBack.setPower(-.1);
        robot.DriveRightBack.setPower(-.1);
        robot.DriveRightFront.setPower(-.1);
        robot.DriveLeftFront.setPower(-.1);
        sleep(time);
    }

    public final void Right (int time)
    {
        robot.DriveLeftBack.setPower(.1);
        robot.DriveRightBack.setPower(-.1);
        robot.DriveRightFront.setPower(.1);
        robot.DriveLeftFront.setPower(-.1);
        sleep(time);
    }
}
