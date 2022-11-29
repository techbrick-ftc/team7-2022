package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class FieldCentric extends StarterAuto{
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode()  {
        TelemetryPacket packet = new TelemetryPacket();
        initialize();
        double zeroAngle = 0;
        double speedMod = 1;

        final double DEGPERVOLT = 81.8;
        final double ARMROTATEMAXANGLE = 200.4;
        int armrotate0 = 0;
        final int ARMROTATEMAXTICKS = 4729;

        double position1 = 0;
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        Gamepad cur2 = new Gamepad();
        Gamepad cur1 = new Gamepad();


        boolean grabberOpen = false;


        waitForStart();

        zeroAngle = imu.getAngularOrientation().firstAngle;

        while (opModeIsActive()) {
            packet.put("arm encoder", stringMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            try {
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {

            }


            if (gamepad2.right_trigger != 0) {
               if (stringMotor.getCurrentPosition() < -3500 || stringpot.getVoltage() <= VOLTSSTRINGUP) {
                    stringMotor.setPower(0);
                 } else {
                    stringMotor.setPower(-gamepad2.right_trigger);
                }
            }
            packet.put("string encoder", stringMotor.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);

            if (gamepad2.left_trigger != 0) {
                if (stringpot.getVoltage() >= VOLTSSTRINGDOWN) {
                    stringMotor.setPower(0);
                } else {
                    stringMotor.setPower(gamepad2.left_trigger);
                }
            }

            if (gamepad2.back){
                while(opModeIsActive() && stringpot.getVoltage() <= VOLTSSTRINGDOWN){
                    stringMotor.setPower(0.5);
                }

                while(opModeIsActive() && !armuptouch.isPressed()){
                    armMotor.setPower(0.5);
                }
            }


            if ((gamepad2.left_stick_y < 0 && (armrotate0 - armMotor.getCurrentPosition()  >= ARMROTATEMAXTICKS || armpot.getVoltage() * DEGPERVOLT >= ARMROTATEMAXANGLE))
                || (gamepad2.left_stick_y > 0 && armuptouch.isPressed())){
                armMotor.setPower(0);
            } else {
                armMotor.setPower(gamepad2.left_stick_y);
            }
            packet.put("arm max", armMotor.getCurrentPosition());
            packet.put("arm up touch", armuptouch.isPressed());
            packet.put("zero", armrotate0);
            packet.put("armvolt", armpot.getVoltage());
            packet.put("stringpot", stringpot.getVoltage());

            if (armuptouch.isPressed()){
                armrotate0 = armMotor.getCurrentPosition();
            }

            if (cur2.a && !previousGamepad2.a) {
                if (!grabberOpen) {
                    grabbaServo.setPosition(1);
                    grabberOpen = true;
                }
                else {
                    grabbaServo.setPosition(0);
                    grabberOpen = false;
                }
            }


            if (cur2.dpad_right && !previousGamepad2.dpad_right){
                position1 += 0.10;

            }
            if (cur2.dpad_left && !previousGamepad2.dpad_left){
                position1 -= 0.10;

            }

            if (cur2.dpad_up && !previousGamepad2.dpad_up){
                position1 = 1;

            }

            if (cur2.dpad_down && !previousGamepad2.dpad_down){
                position1 = 0;
            }

            if (position1 > 1){
                position1 = 1;
            }
           else if (position1 < 0){
                position1 = 0;
            }
            wristServo.setPosition(position1);


            telemetry.addData("position", wristServo.getPosition());
            telemetry.addData("end", position1);

            telemetry.update();


            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = imu.getAngularOrientation().firstAngle - zeroAngle;

            if (gamepad1.y) {
                zeroAngle = imu.getAngularOrientation().firstAngle;
            }


            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            packet.put("rotatex", rotX);
            packet.put("rotatey", rotY);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 0.5);

            double frontLeftPower = (rotY + rotX - rx) / denominator;
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;



            if (cur1.x && !previousGamepad1.x){
                speedMod -= 0.1;
            }

            if (cur1.b && !previousGamepad1.b){
                speedMod += 0.1;
            }

            if (speedMod > 1){
                speedMod = 1;
            }
            else if (speedMod <= 0.3){
                speedMod = 0.3;
            }


            frontRight.setPower(frontRightPower * speedMod);  //front
            frontLeft.setPower(frontLeftPower * speedMod);    //left
            backRight.setPower(backRightPower * speedMod);   //right
            backLeft.setPower(backLeftPower * speedMod);   //back


            packet.put("zer", Math.toDegrees(zeroAngle));
            packet.put("imu", Math.toDegrees(imu.getAngularOrientation().firstAngle));

            packet.put("front right mod", frontRightPower * speedMod);
            packet.put("front left mod", frontLeftPower * speedMod);
            packet.put("back right mod", backRightPower * speedMod);
            packet.put("back left mod", backLeftPower * speedMod);
            dashboard.sendTelemetryPacket(packet);

            try {
                previousGamepad2.copy(cur2);
            } catch (RobotCoreException e) {

            }
        }

    }
}
