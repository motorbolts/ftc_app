package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 6705 on 12/22/2015.
 */
public class LiftTest extends OpMode {

    DcMotor rwa; // P0 port 1
    DcMotor rwb; // P0 port 2
    DcMotor liftL; // P1 port 1
    DcMotor liftR; // P1 port 2
    Servo leftComb; // P2 channel 1
    Servo rightComb; // P2 channel 2
    Servo trigL; // P2 channel 3
    Servo trigR; // P2 channel 4
    DcMotor scoopArm; // P3 port 1
    //DcMotor winch; - Winch not currently on robot will be //P3 port 2
    Servo leftCR; // Not on robot- P4 channel 1
    Servo rightCR; //Not on robot- P4 channel 2
    Servo wrist;
    Servo ddspivot; // P4 channel 3
    Servo ddsclaw; // P4 channel 4
    DcMotor lwa; // P5 port 1
    DcMotor lwb; // P5 port 2
    Servo liftTiltLeft;
    Servo liftTiltRight;
    double tiltLeft;
    double tiltRight;

    public void init() {
        lwa = hardwareMap.dcMotor.get("leftwheelA");
        lwb = hardwareMap.dcMotor.get("leftwheelB");
        rwa = hardwareMap.dcMotor.get("rightwheelA");
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);
        scoopArm = hardwareMap.dcMotor.get("scoopArm");
        scoopArm.setDirection(DcMotor.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");
        //rightCR = hardwareMap.servo.get("rightCR");
        leftComb = hardwareMap.servo.get("leftComb");
        rightComb = hardwareMap.servo.get("rightComb");
        trigL = hardwareMap.servo.get("trigL");
        trigR = hardwareMap.servo.get("trigR");
        ddspivot = hardwareMap.servo.get("ddspivot");
        ddsclaw = hardwareMap.servo.get("ddsclaw");
    }

    public void loop() {

        double lift = gamepad2.left_stick_y;

        lift = Range.clip(lift, -1, 1);

        liftL.setPower(lift);
        liftR.setPower(lift);

        if(gamepad2.right_stick_y > 0.05)
        {
            tiltLeft = tiltLeft + 0.05;
            tiltRight = tiltRight - 0.05;
            liftTiltLeft.setPosition(tiltLeft);
            liftTiltRight.setPosition(tiltRight);

        }

        if(gamepad2.right_stick_y < -0.05)
        {
            tiltLeft = tiltLeft - 0.05;
            tiltRight = tiltRight + 0.05;
            liftTiltLeft.setPosition(tiltLeft);
            liftTiltRight.setPosition(tiltRight);
        }

        if(gamepad2.left_trigger > 0.5)
        {
            leftCR.setPosition(1.0);
            rightCR.setPosition(0.0);
        }
        if(gamepad2.right_trigger > 0.5)
        {
            leftCR.setPosition(0.0);
            rightCR.setPosition(1.0);
        }
        if(gamepad2.left_trigger < 0.5 || gamepad2.right_trigger < 0.5)
        {
            leftCR.setPosition(0.5);
            rightCR.setPosition(0.5);
        }


    }
}
