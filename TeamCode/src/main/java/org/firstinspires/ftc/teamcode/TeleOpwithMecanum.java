
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp w/mecanum", group="Teleop")

public class TeleOpwithMecanum extends OpMode{

    Hardware iHexagon       = new Hardware(); // use the class created to define a Pushbot's hardware



    public void init() {

        iHexagon.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }
    @Override
    public void loop() {

        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;



        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        iHexagon.LeftF.setPower(yaw+(vertical+horizontal));
        iHexagon.LeftB.setPower(yaw+(vertical-horizontal));
        iHexagon.RightF.setPower(-yaw+(vertical-horizontal));
        iHexagon.RightB.setPower(-yaw+(vertical+horizontal));


        /*
        leftWI = -gamepad2.right_stick_y;
        rightWI = -gamepad2.right_stick_y;


        iHexagon.leftDrive.setPower(left);
        iHexagon.rightDrive.setPower(right);
        iHexagon.leftWI.setPower(leftWI);
        iHexagon.rightWI.setPower(rightWI);

        // Use gamepad left & right Bumpers to open and close the claw
        // Move both servos to new position.  Assume servos are mirror image of each other.


        // Use gamepad buttons to move the arm up (Y) and down (A)

        // Send telemetry message to signify robot running;
        telemetry.addData("rightWI",  "%.2f", rightWI);
        telemetry.addData("leftWI",  "%.2f", leftWI);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
            telemetry.addData("rightWI",  "%.2f", rightWI);
        telemetry.addData("leftWI",  "%.2f", leftWI);

        */
         
    }


    @Override
    public void stop() {
    }
}

