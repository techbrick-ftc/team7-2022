package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Autonomous(name = "AutoFinal", group = "Auto")
public class AutoFinal extends LinearOpMode {
    public DcMotor left;
    public DcMotor back;
    public DcMotor front;
    public DcMotor right;
    public BNO055IMU imu;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    void drivingCorrectionStraight(double startAngle2, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle2;
        right.setPower(power + difference);
        left.setPower(power - difference);
    }

    void drivingCorrectionLeft(double startAngle, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle;
        front.setPower(power + difference);
        back.setPower(power - difference);
    }


    private double wrap(double theta) {
        double newTheta = theta;
        while (Math.abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }

    boolean shouldStopTurning(double targetAngle) {
        double currentAngle = imu.getAngularOrientation().firstAngle;
        return Math.abs(currentAngle - targetAngle) < .005 * Math.PI;

    }


    void turnRobot(double angle, double speed, boolean clockwise) {
        double directionalSpeed = clockwise ? -speed : speed;
        double targetAngle = wrap(imu.getAngularOrientation().firstAngle + angle);
        while (opModeIsActive() && !shouldStopTurning(targetAngle)) {
            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            back.setPower(directionalSpeed);
            front.setPower(-directionalSpeed);
            right.setPower(-directionalSpeed);
            left.setPower(directionalSpeed);
        }

        back.setPower(0);
        front.setPower(0);
        right.setPower(0);
        left.setPower(0);

    }

    @Override
    public void runOpMode() {

        TelemetryPacket packet = new TelemetryPacket();
        left = hardwareMap.get(DcMotor.class, "left");
        back = hardwareMap.get(DcMotor.class, "back");
        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        ColorSensor color1 = hardwareMap.colorSensor.get("color1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        waitForStart();

        double startAngle = imu.getAngularOrientation().firstAngle;

        //  packet.addLine("Ticks Before " + front.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        int frontpos = front.getCurrentPosition();
        //Continue moving until robot is on tape
        while (opModeIsActive() && color1.red() < 400 && front.getCurrentPosition() - frontpos < 5500) {
            drivingCorrectionLeft(startAngle, 0.5);
            //  packet.addLine("Red Color " + color1.red());
            packet.put("Red", color1.red());
            packet.put("Ticks", front.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        packet.addLine("Ticks After " + front.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
        back.setPower(0);
        front.setPower(0);
        sleep(1000);


        double startAngle2 = imu.getAngularOrientation().firstAngle;

        right.setPower(0.5);
        left.setPower(0.5);


        double startTime = getRuntime();
        //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
        // this is so robot is for sure off the tape
        int leftpos = left.getCurrentPosition();

        while (opModeIsActive() && ((color1.red() < 400 && left.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
            drivingCorrectionStraight(startAngle2, 0.5);
            // packet.addLine("Runtime" + (getRuntime() - startTime));
            //  packet.addLine("Color Red" + color1.red());
            packet.put("Red", color1.red());
            packet.put("Ticks", left.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        right.setPower(0);
        left.setPower(0);
        sleep(1000);

        turnRobot(-Math.PI / 2, 0.3, true);

        while(opModeIsActive()){


        }
    }

    }
