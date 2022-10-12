package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {
    public DcMotor left;
    public DcMotor back;
    public DcMotor front;
    public DcMotor right;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        back = hardwareMap.get(DcMotor.class, "back");
        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        ColorSensor color1 = hardwareMap.colorSensor.get("color1");
        waitForStart();

        telemetry.addData("red", color1.red());
        telemetry.addData("blue", color1.blue());
        telemetry.addData("green", color1.green());
        telemetry.addData("alpha", color1.alpha());
        telemetry.update();

        //left.setPower(-1);
        // back.setPower(1);
        //front.setPower(-1);
        // right.setPower(1);

        while (color1.alpha() < 15) {
            back.setPower(1);
            front.setPower(-1);
        }

        back.setPower(0);
        front.setPower(0);
        sleep(500);

        while (color1.alpha() < 15) {
            right.setPower(-1);
            left.setPower(1);
        }

        back.setPower(0);
        front.setPower(0);



    }
    }
