/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

public class BACKRedDDS_NSR_SETPICK extends LinearOpMode {

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

    ModernRoboticsI2cGyro Gyro;

    ElapsedTime timer;

    ColorSensor colorSensor;

    OpticalDistanceSensor lineSensor;

    TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {

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
        Gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");

        lineSensor = hardwareMap.opticalDistanceSensor.get("dist1");

        touch = hardwareMap.touchSensor.get("touch");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //***INIT***//

        dump.setPosition(0);
        swivel.setPosition(1);

        trigL.setPosition(0.8);
        trigR.setPosition(0.05);

        dds.setPosition(1);

        holdL.setPosition(0.75);
        holdR.setPosition(0.05);

        holdC.setPosition(1);

        double heading;
        double targetHeading;
        double headingError;
        double drivesteering;

        double leftPower;
        double rightPower;

        Gyro.calibrate();


        // wait for the start button to be pressed
        waitForStart();

        timer = new ElapsedTime();

        collector.setPower(0);

        double midPower = -0.75;
        double driveGain = 0.0875;

        while(lwa.getCurrentPosition() > -9800 && timer.time() < 15)
        {
            waitOneFullHardwareCycle();

            telemetry.addData("Encoder Value", rwa.getCurrentPosition());

            heading = Gyro.getIntegratedZValue();

            telemetry.addData("Heading", heading);

            drivesteering = driveGain * heading;

            rightPower = midPower - drivesteering;
            leftPower = midPower + drivesteering;

            if(leftPower > 1.0)
            {
                leftPower = 1.0;
            }
            if(rightPower < -1.0)
            {
                rightPower = -1.0;
            }

            rightPower = Range.clip(rightPower, -1, 1);
            leftPower = Range.clip(leftPower, -1, 1);

            telemetry.addData("leftPower", leftPower);
            telemetry.addData("rightPower", rightPower);

            lwa.setPower(leftPower);
            lwb.setPower(leftPower);
            rwa.setPower(rightPower);
            rwb.setPower(rightPower);
        }

        waitOneFullHardwareCycle();

        rwa.setPower(0);
        rwb.setPower(0);
        lwa.setPower(0);
        lwb.setPower(0);

        sleep(100);

        heading = Gyro.getIntegratedZValue();

        sleep(100);

        midPower = 0;
        double minPowerPositive = 0.2;
        double minPowerNegative = -0.2;
        driveGain = 0.005;

        while((heading < -134 || heading > -134) && timer.time() < 15)
        {
            waitOneFullHardwareCycle();
            heading = Gyro.getIntegratedZValue();
            telemetry.addData("zheading", heading);

            while(heading > -134 && timer.time() < 15) {

                waitOneFullHardwareCycle();

                heading = Gyro.getIntegratedZValue();

                targetHeading = -134;

                headingError = targetHeading - heading;

                drivesteering = Math.abs(driveGain * headingError);

                if (drivesteering > 1) {
                    drivesteering = 1;
                    telemetry.addData("Caught illegal value", "reset drivesteering to 1");
                }

                leftPower = midPower + drivesteering;
                rightPower = midPower - drivesteering;

                if (leftPower < minPowerPositive) {
                    leftPower = minPowerPositive;
                }
                if (rightPower > minPowerNegative) {
                    rightPower = minPowerNegative;
                }

                lwa.setPower(leftPower);
                lwb.setPower(leftPower);
                rwa.setPower(rightPower);
                rwb.setPower(rightPower);
            }

            while(heading < -134 && timer.time() < 15)
            {
                waitOneFullHardwareCycle();

                heading = Gyro.getIntegratedZValue();

                targetHeading = -134;

                headingError = targetHeading - heading;

                drivesteering = Math.abs( driveGain * headingError);

                if(drivesteering > 1)
                {
                    drivesteering = 1;
                }

                rightPower = midPower + drivesteering;
                leftPower = midPower - drivesteering;

                if(rightPower < minPowerPositive)
                {
                    rightPower = minPowerPositive;
                }

                if(leftPower > minPowerNegative)
                {
                    leftPower = minPowerNegative;
                }

                lwa.setPower(leftPower);
                lwb.setPower(leftPower);
                rwa.setPower(rightPower);
                rwb.setPower(rightPower);
            }
        }

        waitOneFullHardwareCycle();

        lwa.setPower(0);
        lwb.setPower(0);
        rwa.setPower(0);
        rwb.setPower(0);
        sleep(100);

        lwa.setPower(0.75);
        lwb.setPower(0.75);
        rwa.setPower(0.75);
        rwb.setPower(0.75);
        sleep(500);



        while (!touch.isPressed() && timer.time() < 15) {

            if ((colorSensor.blue()<4)) {

                lwa.setPower(0.5);
                lwb.setPower(0.5);
                rwa.setPower(0);
                rwb.setPower(0);
            } else {
                lwa.setPower(0);
                lwb.setPower(0);
                rwa.setPower(.50);
                rwb.setPower(.50);
            }
        }
        waitOneFullHardwareCycle();

        lwa.setPower(0.0);
        lwb.setPower(0.0);
        rwa.setPower(0.0);
        rwb.setPower(0.0);
        sleep(100);


        collector.setPower(0.0);
        sleep(100);

        if(timer.time() < 15) {
            lwa.setPower(-0.35);
            lwb.setPower(-0.35);
            rwa.setPower(-0.35);
            rwb.setPower(-0.35);

            sleep(225);

            lwa.setPower(0.0);
            lwb.setPower(0.0);
            rwa.setPower(0.0);
            rwb.setPower(0.0);

            sleep(100);
            dds.setPosition(0);

            sleep(900);
        }

        midPower = -.5;

        while(heading < -85)
        {
            waitOneFullHardwareCycle();
            heading = Gyro.getIntegratedZValue();
            telemetry.addData("zheading", heading);

            waitOneFullHardwareCycle();

            heading = Gyro.getIntegratedZValue();

            targetHeading = -85;

            headingError = targetHeading - heading;

            drivesteering = Math.abs(driveGain * headingError);

            if (drivesteering > 1) {
                drivesteering = 1;
                telemetry.addData("Caught illegal value", "reset drivesteering to 1");
            }

            leftPower = midPower - drivesteering;

            if (leftPower > minPowerNegative) {
                leftPower = minPowerNegative;
            }

            lwa.setPower(leftPower);
            lwb.setPower(leftPower);
        }

        lwa.setPower(0.0);
        lwb.setPower(0.0);
        rwa.setPower(0.0);
        rwb.setPower(0.0);
        sleep(100);

        lwa.setPower(-.80);
        lwb.setPower(-.80);
        rwa.setPower(-.80);
        rwb.setPower(-.80);
        sleep(1700);

        lwa.setPower(0.0);
        lwb.setPower(0.0);
        rwa.setPower(0.0);
        rwb.setPower(0.0);
        sleep(100);

    }
}
