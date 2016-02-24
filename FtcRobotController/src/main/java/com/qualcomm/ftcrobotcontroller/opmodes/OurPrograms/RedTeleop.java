package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

public class RedTeleop extends OpMode {

    DcMotor rwa; //rightwheels
    DcMotor rwb;

    DcMotor lwa; //leftwheels
    DcMotor lwb;

    DcMotor liftL;
    DcMotor liftR;

    Servo dump; //servo to score with dumper
    Servo swivel; //servo that 'spreads' with dumper

    Servo holdR; // churro holders
    Servo holdL;

    Servo trigL; //zip-line triggers
    Servo trigR;

    Servo holdC; //servo to guard lift

    DcMotor collector; //bottom collector motor
    DcMotor collector2; //top collector motor

    Servo dds;

    @Override
    public void init() {

        lwa = hardwareMap.dcMotor.get("leftwheelA"); //leftwheels
        lwb = hardwareMap.dcMotor.get("leftwheelB");

        rwa = hardwareMap.dcMotor.get("rightwheelA"); //rightwheels
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);

        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);

        collector = hardwareMap.dcMotor.get("collector");
        collector2 = hardwareMap.dcMotor.get("collector2");

        dump = hardwareMap.servo.get("dump"); //dumper control
        swivel = hardwareMap.servo.get("swivel");

        trigL = hardwareMap.servo.get("trigL"); //triggers
        trigR = hardwareMap.servo.get("trigR");

        dds = hardwareMap.servo.get("dds");

        holdL = hardwareMap.servo.get("holdL"); //churro holder
        holdR = hardwareMap.servo.get("holdR");

        holdC = hardwareMap.servo.get("holdC"); //lift holder

        //***INIT***//

        dump.setPosition(0);
        swivel.setPosition(1);

        trigL.setPosition(0.8);
        trigR.setPosition(0.05);

        dds.setPosition(1);

        holdL.setPosition(0.75);
        holdR.setPosition(0.05);

        holdC.setPosition(1);
    }

    double swivelVal = 1;

    @Override
    public void loop() {

// wheel control
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        rwa.setPower(right);
        rwb.setPower(right);
        lwa.setPower(left);
        lwb.setPower(left);

// lift control
        float lift = gamepad2.right_stick_y;

        lift = Range.clip(lift, -1, 1);

        if (lift >= 0) {
            liftL.setPower(lift / 5);
            liftR.setPower(lift / 5);
        }
        if (lift < 0) {
            liftL.setPower(lift);
            liftR.setPower(lift);
        }

//collector control
        if (gamepad2.b) {
            collector.setPower(1);
            collector2.setPower(-1);
        } else if (gamepad2.a) {
            collector.setPower(-1);
            collector2.setPower(1);
        } else {
            collector.setPower(0);
            collector2.setPower(0);
        }

// Zipline release control
        if (gamepad1.left_bumper) {
            trigL.setPosition(0);
        } else {
            trigL.setPosition(0.8);
        }

        if (gamepad1.right_bumper) {
            trigR.setPosition(0.8);
        } else {
            trigR.setPosition(0.05);
        }


// Dumper scoring
        if (gamepad2.right_trigger > 0.5) {
            dump.setPosition(1);
        } else {
            dump.setPosition(0);
        }

//Dumper presets
        if (gamepad2.left_stick_button) {
            swivel.setPosition(1);
            swivelVal = 0.99;
        }
        //straight up
        if (gamepad2.right_stick_button) {
            swivel.setPosition(0.5);
            swivelVal = 0.5;
        }

//Dumper adjustment
        swivelVal = Range.clip(swivelVal, 0, 1);
        swivelVal = Range.clip(swivelVal, 0, 1);

        if (gamepad2.left_stick_y > 0.25 && (swivelVal < 1)) {
            swivelVal = Range.clip(swivelVal, 0, 1);
            swivelVal = Range.clip(swivelVal, 0.01, 1);
            swivelVal = swivelVal + 0.01;
            swivel.setPosition(swivelVal);
        }

        if (gamepad2.left_stick_y < -0.25 && (0 < swivelVal)) {
            swivelVal = Range.clip(swivelVal, 0, 1);
            swivelVal = Range.clip(swivelVal, 0, 1);
            swivelVal = swivelVal - 0.01;
            swivel.setPosition(swivelVal);
        }

//scooch button control
        if (gamepad1.y) {
            lwa.setPower(0.20);
            lwb.setPower(0.20);
            rwa.setPower(0.20);
            rwb.setPower(0.20);
        }

        if (gamepad1.a) {
            lwa.setPower(-0.20);
            lwb.setPower(-0.20);
            rwa.setPower(-0.20);
            rwb.setPower(-0.20);
        }

        if (gamepad1.b) {
            lwa.setPower(-0.50);
            lwb.setPower(-0.50);
            rwa.setPower(0.50);
            rwb.setPower(0.50);
        }

        if (gamepad1.x) {
            lwa.setPower(0.50);
            lwb.setPower(0.50);
            rwa.setPower(-0.50);
            rwb.setPower(-0.50);
        }

//churro grabbers
        if (gamepad2.left_bumper)  //release churros
        {
            holdL.setPosition(0.75);
            holdR.setPosition(0.05);
            holdC.setPosition(1);
        }
        if (gamepad2.right_bumper)  //hold churro
        {
            holdL.setPosition(0.1);
            holdR.setPosition(0.70);
        }

//lift guard
        if (gamepad2.x) //midway button
        {
            //   holdL.setPosition(0.47);
            //  holdR.setPosition(0.15);
            holdC.setPosition(0.25);
        }

//score climbers
        if (gamepad2.x) {
            dds.setPosition(0);
        }


		/*
         * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));


    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	/*
	@Override
	public void stop() {

	}
	

	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.

	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}
		
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}
		
		return dScale;
	}
*/
}
