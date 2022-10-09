package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorTest extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        ColorSensor color1 = hardwareMap.colorSensor.get("color1");
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("red", color1.red());
            telemetry.addData("blue", color1.blue());
            telemetry.addData("green", color1.green());
            telemetry.addData("alpha", color1.alpha());
            telemetry.update();
        }
    }
}
