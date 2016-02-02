
package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;
//
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class ColorTest extends OpMode {

    ColorSensor colorSensor;
    @Override
    public void init() {

        colorSensor = hardwareMap.colorSensor.get("colorSensor");



    }

    @Override
    public void loop() {

if(colorSensor.blue() > 1)
{
    telemetry.addData("Blue ", colorSensor.blue());
}

        if(colorSensor.red() > 1)
        {
            telemetry.addData("Red ", colorSensor.red());
        }
        if(colorSensor.green() > 1)
        {
            telemetry.addData("Green ", colorSensor.green());
        }
        if(colorSensor.alpha() > 1)
        {
            telemetry.addData("Clear ", colorSensor.alpha());
        }




    }
}
