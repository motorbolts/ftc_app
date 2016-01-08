
package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;
//
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class BlueTeleop extends OpMode {

    DcMotor rwa; // P0 port 1
    DcMotor rwb; // P0 port 2
    DcMotor liftL; // P1 port 1
    DcMotor liftR; // P1 port 2
    Servo leftComb; // P2 channel 1
    Servo rightComb; // P2 channel 2
    Servo trigL; // P2 channel 3
    Servo trigR; // P2 channel 4
    DcMotor collector; // P3 port 1
    Servo leftCR; // Not on robot- P4 channel 1
    Servo rightCR; //Not on robot- P4 channel 2
    Servo dds; // P4 channel 3
    DcMotor lwa; // P5 port 1
    DcMotor lwb; // P5 port 2
   // Servo leftPivot;
   // Servo rightPivot;


    @Override
    public void init() {

        lwa = hardwareMap.dcMotor.get("leftwheelA");
        lwb = hardwareMap.dcMotor.get("leftwheelB");
        rwa = hardwareMap.dcMotor.get("rightwheelA");
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);
//        liftL = hardwareMap.dcMotor.get("liftL");
  //      liftR = hardwareMap.dcMotor.get("liftR");
    //    liftR.setDirection(DcMotor.Direction.REVERSE);
      //  liftL.setDirection(DcMotor.Direction.REVERSE);
        collector = hardwareMap.dcMotor.get("collector");
      //  rightCR = hardwareMap.servo.get("rightCR");
      //  leftCR = hardwareMap.servo.get("leftCR");
        leftComb = hardwareMap.servo.get("leftComb");
        rightComb = hardwareMap.servo.get("rightComb");
        trigL = hardwareMap.servo.get("trigL");
        trigR = hardwareMap.servo.get("trigR");
        dds = hardwareMap.servo.get("dds");
      //  leftPivot = hardwareMap.servo.get("leftPivot");
      //  rightPivot = hardwareMap.servo.get("rightPivot");


      //  leftPivot.setPosition(1);
       // rightPivot.setPosition(0);
        leftComb.setPosition(0);
        rightComb.setPosition(1);
        trigL.setPosition(0.7);
        trigR.setPosition(0.35);
     //   leftCR.setPosition(0.5);
   //     rightCR.setPosition(0.5);
        dds.setPosition(1);
    }

 //   double lPivot = 0;
  //  double rPivot = 1;


    @Override
    public void loop() {



// wheel control
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        rwa.setPower(right);
        rwb.setPower(right);
        lwa.setPower(left);
        lwb.setPower(left);

// lift control
      //  float lift = gamepad2.left_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
//        lift = Range.clip(lift, -1, 1);

  //      liftL.setPower(lift);
//        liftR.setPower(lift);


        if (gamepad2.b){
            collector.setPower(1);//yo
        }
        else if (gamepad2.a)
        {
            collector.setPower(-1);
        }

        else {
            collector.setPower(0);
        }

        //collector vertical control
/*
        float collLift = gamepad2.right_stick_y;
        float collLiftPower = ((collLift + 1)/2);

        leftCR.setPosition(collLiftPower);
        rightCR.setPosition(Math.abs(collLiftPower - 1));
*/

// Zipline release control
		if(gamepad2.left_bumper) {
		trigL.setPosition(0.05);
		}
		else{
			trigL.setPosition(0.7);
		}


        if(gamepad2.right_bumper) {
            trigR.setPosition(1);
        }
        else{
            trigR.setPosition(0.35);
        }



// Comb control

		if(gamepad2.left_trigger>0.5)
		{
			rightComb.setPosition(0);
		}
		else
		{
			rightComb.setPosition(1);
		}


        if(gamepad2.right_trigger > 0.5)
        {
            rightComb.setPosition(0);
        }
        else
        {
            rightComb.setPosition(1);
        }


        if(gamepad1.y)
        {
            lwa.setPower(0.20);
            lwb.setPower(0.20);
            rwa.setPower(0.20);
            rwb.setPower(0.20);
        }

        if(gamepad1.a)
        {
            lwa.setPower(-0.20);
            lwb.setPower(-0.20);
            rwa.setPower(-0.20);
            rwb.setPower(-0.20);
        }

        if(gamepad1.b)
        {
            lwa.setPower(-0.50);
            lwb.setPower(-0.50);
            rwa.setPower(0.50);
            rwb.setPower(0.50);
        }

        if(gamepad1.x)
        {
            lwa.setPower(0.50);
            lwb.setPower(0.50);
            rwa.setPower(-0.50);
            rwb.setPower(-0.50);
        }

       // rPivot = Range.clip(rPivot, 0.01, 0.99);
       // lPivot = Range.clip(lPivot, 0.01, 0.99);

       /* if(gamepad2.right_bumper)
        {
            lPivot = (lPivot + 0.01);
            leftPivot.setPosition(lPivot);
            rPivot = (rPivot - 0.01);
            rightPivot.setPosition(rPivot);
        }

        if(gamepad2.left_bumper)
        {
            lPivot = (lPivot - 0.01);
            leftPivot.setPosition(lPivot);
            rPivot = (rPivot + 0.01);
            rightPivot.setPosition(rPivot);
        }

*/

        if(gamepad2.x)
        {
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
