package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class MainTeleOp extends StarterAuto {

    enum states {
        Manual, GrabAlign, Grab, Align, Release, Pause
    }

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize();
        double zeroAngle = 0;
        boolean speedMod = true;

        final double DEGPERVOLT = 81.8;

        int armrotate0 = 0;

        boolean fieldCentric = true;
        double stringPotLastVal = stringpot.getVoltage();

        states currentState = states.Manual;
        states pausedState = states.Pause;

        double rotX = 0;
        double rotY = 0;
        double rx = 0;

        double armGrabPos = ARMVOLTSMID;
        double stringGrabPos = VOLTSSTRINGDOWN;

        double armDropPos = ARMVOLTSMID;
        double stringDropPos = VOLTSSTRINGDOWN;

        double wristPosition = 0;
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        Gamepad cur2 = new Gamepad();
        Gamepad cur1 = new Gamepad();

        boolean grabberOpen = true;

        waitForStart();

        zeroAngle = imu.getAngularOrientation().firstAngle;

        while (opModeIsActive()) {
            packet.put("arm encoder", stringMotor.getCurrentPosition());
            packet.put("state", currentState);
            dashboard.sendTelemetryPacket(packet);

            try {
                cur2.copy(gamepad2);
                cur1.copy(gamepad1);
            } catch (RuntimeException e) {

            }

            // right trigger extends, extending is -1  power
            if (currentState == states.Manual) {
                if (gamepad2.right_trigger > 0) {

                    if (stringpot.getVoltage() <= VOLTSSTRINGUP) {
                        stringMotor.setPower(0);
                        stringPotLastVal = stringpot.getVoltage();
                    } else {
                        stringMotor.setPower(-gamepad2.right_trigger);
                        stringPotLastVal = stringpot.getVoltage();
                    }
                } else if (gamepad2.left_trigger > 0) {
                    if (stringpot.getVoltage() >= VOLTSSTRINGDOWN) {
                        stringMotor.setPower(0);
                        stringPotLastVal = stringpot.getVoltage();

                    } else {
                        stringMotor.setPower(gamepad2.left_trigger);
                        stringPotLastVal = stringpot.getVoltage();
                    }
                } else {
                    if (armpot.getVoltage() > 2) {
                        stringMotor.setPower(0);
                    } else {
                        stringMotor.setPower(-0.1);
                    }
                }
            }
            packet.put("last val", stringPotLastVal);
            dashboard.sendTelemetryPacket(packet);

            if (cur2.back && !previousGamepad2.back) {
                stringSync(0.368);
            }

            if (currentState == states.Manual) {
                if ((gamepad2.left_stick_y < 0 && armpot.getVoltage() >= ARMROTATEMAXVOLT)
                        || (gamepad2.left_stick_y > 0 && armuptouch.isPressed())) {
                    armMotor.setPower(0);
                } else {
                    armMotor.setPower(gamepad2.left_stick_y * 0.7);
                }
                packet.put("arm max", armMotor.getCurrentPosition());
                packet.put("arm up touch", armuptouch.isPressed());
                packet.put("zero", armrotate0);
                packet.put("armvolt", armpot.getVoltage());
                packet.put("stringpot", stringpot.getVoltage());
            }

            if (armuptouch.isPressed()) {
                armrotate0 = armMotor.getCurrentPosition();
            }

            if (cur2.a && !previousGamepad2.a) {
                if (!grabberOpen) {
                    grabbaServo.setPosition(0.3);
                    grabberOpen = true;
                    armDropPos = armpot.getVoltage();
                    stringDropPos = stringpot.getVoltage();

                } else {
                    grabbaServo.setPosition(1);
                    grabberOpen = false;
                    armGrabPos = armpot.getVoltage();
                    stringGrabPos = stringpot.getVoltage();
                }
            }

            if (!cur2.y && previousGamepad2.y) {
                if (currentState == states.Manual) {
                    currentState = states.GrabAlign;
                    wristServo.setPosition(0);
                } else if (currentState == states.Pause) {
                    currentState = pausedState;
                }
            }
            if (cur2.y && !previousGamepad2.y) {
                if (currentState == states.GrabAlign || currentState == states.Align) {
                    pausedState = currentState;
                    currentState = states.Pause;
                }
            }
            if (cur2.b && !previousGamepad2.b) {
                stringMotor.setPower(0);
                armMotor.setPower(0);
                currentState = states.Manual;
                pausedState = states.Manual;

            }
            if (currentState == states.GrabAlign) {
                boolean armDone = armAsync(armGrabPos - 0.5,false);
                boolean stringDone = stringAsync(stringGrabPos);
                if (armDone && stringDone) {
                    currentState = states.Grab;
                }
            }
            if (currentState == states.Grab) {
                grabbaServo.setPosition(0.3);
                armSync(armGrabPos);
                sleep(200);
                grabbaServo.setPosition(1);
                sleep(200);
                wristServo.setPosition(0.94);
                currentState = states.Align;
            }
            if (currentState == states.Align) {
                boolean armDone = armAsync(armDropPos + 0.3, false);

                if (armDone) {
                    currentState = states.Release;
                }
            }
            if (currentState == states.Release) {
                stringSync(stringDropPos);
                armSync(armDropPos);
                sleep(200);
                grabbaServo.setPosition(0.3);
                sleep(200);
                armSync(1);
                wristServo.setPosition(0);
                currentState = states.Manual;
            }
            if (currentState == states.Pause) {
                stringMotor.setPower(0);
                armMotor.setPower(0);
            }


            if (currentState == states.Manual) {
                if (cur2.dpad_right && !previousGamepad2.dpad_right) {
                    wristPosition += 0.1;

                }
                if (cur2.dpad_left && !previousGamepad2.dpad_left) {
                    wristPosition -= 0.1;

                }

                if (cur2.right_stick_y < -0.9) {
                    wristPosition = 0.94;
                }

                if (cur2.right_stick_y > 0.9) {
                    wristPosition = 0;
                }

                if (wristPosition > 1) {
                    wristPosition = 0.94;
                } else if (wristPosition < 0) {
                    wristPosition = 0;
                }
                wristServo.setPosition(wristPosition);
                packet.put("position", wristPosition);
            }
            if (cur1.right_bumper && !previousGamepad1.right_bumper) {
                speedMod = true;
            } else if (cur1.left_bumper && !previousGamepad1.left_bumper) {
                speedMod = false;
            }

            telemetry.addData("position", wristServo.getPosition());
            telemetry.addData("end", wristPosition);
            telemetry.addData("speedMod", speedMod);
            telemetry.update();

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;

            if (!speedMod) {
                y = Range.clip(-gamepad1.left_stick_y, -0.4, 0.4);
                x = Range.clip(gamepad1.left_stick_x, -0.4, 0.4);
                rx = Range.clip(gamepad1.right_stick_x, -0.25, 0.25);
            } else {
                y = Range.clip(-gamepad1.left_stick_y, -0.95, 0.95);
                rx = Range.clip(gamepad1.right_stick_x, -0.75, 0.75);
            }

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -(imu.getAngularOrientation().firstAngle - zeroAngle);

            if (gamepad1.y) {
                zeroAngle = imu.getAngularOrientation().firstAngle;
            }
            if (cur1.b && !previousGamepad1.b) {
                fieldCentric = !fieldCentric;
            }

            if (fieldCentric) {
                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            } else {
                rotX = x;
                rotY = y;
            }

            packet.put("rotatex", rotX);
            packet.put("rotatey", rotY);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

//            double frontLeftPower = (rotY + rotX - rx) / denominator;
//            double backLeftPower = (rotY - rotX - rx) / denominator;
//            double frontRightPower = (rotY - rotX + rx) / denominator;
//            double backRightPower = (rotY + rotX + rx) / denominator;

            double frontLeftPower = (rotY + rotX - rx);
            double backLeftPower = (rotY - rotX - rx);
            double frontRightPower = (rotY - rotX + rx);
            double backRightPower = (rotY + rotX + rx);


            frontRight.setPower(frontRightPower);  // front
            frontLeft.setPower(frontLeftPower);    // left
            backRight.setPower(backRightPower);    // right
            backLeft.setPower(backLeftPower);      // back

            packet.put("zer", Math.toDegrees(zeroAngle));
            packet.put("imu", Math.toDegrees(imu.getAngularOrientation().firstAngle));

            dashboard.sendTelemetryPacket(packet);

            try {
                previousGamepad1.copy(cur1);
                previousGamepad2.copy(cur2);
            } catch (RuntimeException e) {
            }
        }
    }
}