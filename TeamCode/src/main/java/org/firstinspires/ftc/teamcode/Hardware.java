package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  LeftF   = null;
    public DcMotor LeftB = null;
    public DcMotor  RightF  = null;
    public DcMotor  RightB  = null;
    public DcMotor  leftWI     = null;
    public DcMotor  rightWI = null;




    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftF = hwMap.get(DcMotor.class, "LF");
        LeftB = hwMap.get(DcMotor.class, "LB");
        RightF = hwMap.get(DcMotor.class, "RF");
        RightB = hwMap.get(DcMotor.class, "RB");
        leftWI    = hwMap.get(DcMotor.class, "lwi");
        rightWI = hwMap.get(DcMotor.class, "rwi");

        LeftF.setDirection(DcMotor.Direction.FORWARD);
        LeftB.setDirection(DcMotor.Direction.FORWARD);
        RightF.setDirection(DcMotor.Direction.REVERSE);
        RightB.setDirection(DcMotor.Direction.REVERSE);
        leftWI.setDirection(DcMotor.Direction.FORWARD);
        rightWI.setDirection(DcMotor.Direction.REVERSE);


        LeftF.setPower(0);
        LeftB.setPower(0);
        RightF.setPower(0);
        RightB.setPower(0);

        leftWI.setPower(0);
        rightWI.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.


    }
 }

