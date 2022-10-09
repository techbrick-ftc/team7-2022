package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {
    public DcMotor left;
    public DcMotor back;
    public DcMotor front;
    public DcMotor right;

    @Override
    public void runOpMode() {
        left  = hardwareMap.get(DcMotor.class, "left");
        back = hardwareMap.get(DcMotor.class, "back");
        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");

        waitForStart();

       left.setPower(-1);
       back.setPower(1);
       front.setPower(-1);
       right.setPower(1);

        sleep(2000);

        left.setPower(0);
        back.setPower(0);
        front.setPower(0);
        right.setPower(0);

    }
}