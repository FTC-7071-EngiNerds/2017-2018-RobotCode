package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//Declare Opmode
@TeleOp(name="TeleOp", group="Enginerds")
//@Disabled
public class TeleOp1 extends LinearOpMode {

    /* Declare OpMode members. */
    //Import Hardware
    //Anything with robot in front is called from hardware
    TeleOpHardware1 robot           = new TeleOpHardware1();              // Use a K9's hardware

    @Override
    public void runOpMode() {
        double yAxis;
        double xAxis;
        
        //Multi-speed variables
        int Resistor;
        int TurnResistor;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //Jewel Arm Up
        robot.JewelArm.setPosition(0);
        
        //Sensor off
        robot.JewelSensor.enableLed(false);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Variables for drive, take joystick inputs
            yAxis = -gamepad1.left_stick_y;
            xAxis = gamepad1.left_stick_x;

            //Speed control for drive
            //Both Triggers = Fast
            if (gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1)
            {
                Resistor = 1;
                TurnResistor = -1;
            }
            //Both Bumpers = Slow
            else if (gamepad1.right_bumper && gamepad1.right_bumper)
            {
                Resistor = 4;
                TurnResistor = -4;
            }
            // Anything else = Normal
            else
            {
                Resistor = 2;
                TurnResistor = -2;
            }

            //Turn
            //As long as right joystick, which controls turning, is moving, turn
            if(!gamepad1.b && !gamepad1.x && gamepad1.right_stick_x != 0)
            {
                //Right and Left go opposite directions
                robot.DriveLeftBack.setPower(gamepad1.right_stick_x/TurnResistor);
                robot.DriveRightBack.setPower(-gamepad1.right_stick_x/TurnResistor);
                robot.DriveRightFront.setPower(-gamepad1.right_stick_x/TurnResistor);
                robot.DriveLeftFront.setPower(gamepad1.right_stick_x/TurnResistor);
            }

            //Hard Turn Right
            if(gamepad1.b && gamepad1.right_stick_x == 0 && !gamepad1.x)
            {
                //Same as turn but faster right turn due to slowed right side
                robot.DriveLeftBack.setPower(gamepad1.right_stick_x);
                robot.DriveRightBack.setPower(-gamepad1.right_stick_x/10);
                robot.DriveRightFront.setPower(-gamepad1.right_stick_x/10);
                robot.DriveLeftFront.setPower(gamepad1.right_stick_x);
            }
            //Hard Turn Left
            if(gamepad1.x && gamepad1.right_stick_x == 0 && !gamepad1.b)
            {
                //Same as turn but faster left turn due to slowed left side
                robot.DriveLeftBack.setPower(gamepad1.right_stick_x/10);
                robot.DriveRightBack.setPower(-gamepad1.right_stick_x);
                robot.DriveRightFront.setPower(-gamepad1.right_stick_x);
                robot.DriveLeftFront.setPower(gamepad1.right_stick_x/10);
            }

            //Left joystick controls driving, right joystick controls turning

            //Moving Forward
            //As long as not turning and left joystick is within a forward facing range
            if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y < -.10 &&
                    gamepad1.left_stick_x < .30 && gamepad1.left_stick_x > -.30)
            {
                //All motors forward
                robot.DriveLeftBack.setPower(yAxis/Resistor);
                robot.DriveRightBack.setPower(yAxis/Resistor);
                robot.DriveRightFront.setPower(yAxis/Resistor);
                robot.DriveLeftFront.setPower(yAxis/Resistor);
            }

            //Moving Diagonal Left Forward
            //As long as not turning and left joystick is within a diagonal left forward range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y < -.30 && gamepad1.left_stick_x < -.30)
            {
                robot.DriveLeftBack.setPower((-xAxis + yAxis)/Resistor);
                robot.DriveRightBack.setPower(0);
                robot.DriveRightFront.setPower((-xAxis + yAxis)/Resistor);
                robot.DriveLeftFront.setPower(0);
            }

            //Moving Left
            //As long as not turning and left joystick is within a left range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_x < -.10 &&
                    gamepad1.left_stick_y < .30 && gamepad1.left_stick_y > -.30)
            {
                robot.DriveLeftBack.setPower(-xAxis/Resistor);
                robot.DriveRightBack.setPower(xAxis/Resistor);
                robot.DriveRightFront.setPower(-xAxis/Resistor);
                robot.DriveLeftFront.setPower(xAxis/Resistor);
            }

            //Moving Diagonal Left Backwards
            //As long as not turning and left joystick is within a diagonal left back range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y > .30 && gamepad1.left_stick_x < -.30)
            {
                robot.DriveLeftBack.setPower(0);
                robot.DriveRightBack.setPower((xAxis + yAxis)/Resistor);
                robot.DriveRightFront.setPower(0);
                robot.DriveLeftFront.setPower((xAxis + yAxis)/Resistor);
            }

            //Moving Backward
            //As long as not turning and left joystick is within a backwards range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y > .10 &&
                    gamepad1.left_stick_x < .30 && gamepad1.left_stick_x > -.30)
            {
                robot.DriveLeftBack.setPower(yAxis/Resistor);
                robot.DriveRightBack.setPower(yAxis/Resistor);
                robot.DriveRightFront.setPower(yAxis/Resistor);
                robot.DriveLeftFront.setPower(yAxis/Resistor);
            }

            //Moving Diagonal Right Backwards
            //As long as not turning and left joystick is within a diagonal right backwards range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y > .30 && gamepad1.left_stick_x > .30)
            {
                robot.DriveLeftBack.setPower((-xAxis + yAxis)/Resistor);
                robot.DriveRightBack.setPower(0);
                robot.DriveRightFront.setPower((-xAxis + yAxis)/Resistor);
                robot.DriveLeftFront.setPower(0);
            }

            //Moving Right
            //As long as not turning and left joystick is within a right range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_x > .10 &&
                    gamepad1.left_stick_y < .30 && gamepad1.left_stick_y > -.30)
            {
                robot.DriveLeftBack.setPower(-xAxis/Resistor);
                robot.DriveRightBack.setPower(xAxis/Resistor);
                robot.DriveRightFront.setPower(-xAxis/Resistor);
                robot.DriveLeftFront.setPower(xAxis/Resistor);
            }
            //Moving Diagonal Right Forwards
            //As long as not turning and left joystick is within a diagonal right forward range
            else if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y < -.30 && gamepad1.left_stick_x > .30)
            {
                robot.DriveLeftBack.setPower(0);
                robot.DriveRightBack.setPower((xAxis + yAxis)/Resistor);
                robot.DriveRightFront.setPower(0);
                robot.DriveLeftFront.setPower((xAxis + yAxis)/Resistor);
            }

            //If none of the joysticks are touched, do not move
            if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 )
            {
                robot.DriveLeftBack.setPower(0);
                robot.DriveRightBack.setPower(0);
                robot.DriveRightFront.setPower(0);
                robot.DriveLeftFront.setPower(0);
            }

            //Glyph Gripper
            //If left open, if right close
            if (gamepad2.left_bumper)
            {
                robot.Intake.setPower(1);
            }
            else if (gamepad2.right_bumper)
            {
                robot.Intake.setPower(-1);
            }
            else
            {
                robot.Intake.setPower(0);
            }

            //The relic motor's power is based on the operator's right stick location
            robot.RelicExtender.setPower(-gamepad2.left_stick_y);

            //Relic Elbow
            //If a is pressed extend, if not close
            if (gamepad2.a)
            {
                robot.RelicElbow.setPosition(0);
            }
            else if (gamepad2.b)
            {
                robot.RelicElbow.setPosition(.35);
            }
            else
            {
                robot.RelicElbow.setPosition(1);
            }

            //Relic Gripper
            //If left open, if right close
            if (gamepad2.left_trigger == 1)
            {
                robot.RelicGrip.setPosition(.2);
            }
            else if (gamepad2.right_trigger == 1)
            {
                robot.RelicGrip.setPosition(1);
            }

            //Lift
            robot.Lift.setPower(gamepad2.right_stick_y*2/3);

            //Telemetry to check controllers are working as planned
            telemetry.addData("gamepad1.left_stick_y",  "%.2f", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.left_stick_x", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("yAxis",  "%.2f", yAxis);
            telemetry.addData("xAxis", "%.2f", xAxis);

            telemetry.addData("2 Right Bumper", gamepad2.right_bumper);
            telemetry.addData("2 Left Bumper", gamepad2.left_bumper);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}