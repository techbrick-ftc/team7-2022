package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.lang.Math;
/*
Configurations:
Expansion Hub:
 I2C Port 0: colorBL     aka color1
 I2C Port 1: colorBR
 I2C Port 2: colorFR
 I2C Port 3: colorFL

 Servo Port 0:wristServo



 Motor Port 1: armMotor
 Motor Port 0: stringMotor

 Control Hub:
  Motor Port 0: frontRight
  Motor Port 1: frontLeft
  Motor Port 2: backLeft
  Motor Port 3: backRight

  I2C Port 0: imu

  Digital 1: armuptouch

  Analog 0 : stringpot

  Analog 2: armpot

  Servo Port 5:grabbaServo


 */

public class StarterAuto extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    final double VOLTSPERTRIP = 1.438; // may need to change
    final double VOLTSSTRINGUP = 0.92;
    final double VOLTSSTRINGDOWN = 3.17;
    final double TICKSPERBLOCK = 805;   // ~400 per foot
    final double ARMROTATEMAXVOLT = 2.3;
    final double ARMVOLTSMID = 0.95;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public BNO055IMU imu;
    public ColorSensor colorFR;
    public ColorSensor colorFL;
    public ColorSensor colorBR;
    public ColorSensor colorBL;
    public TouchSensor armuptouch;
    public Servo grabbaServo;
    public Servo wristServo;
    public AnalogInput stringpot;
    public AnalogInput armpot;
    public DcMotor armMotor;
    public DcMotor stringMotor;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 1481.603;
    double fy = 1527.539;
    double cx = 550.003;
    double cy = 90.751;
    // UNITS ARE METERS
    double tagsize = 0.045;
    int numFramesWithoutDetection = 0;
    int[] arrayDetections = new int[64];
    int detectionIndex = 0;

    public void insertDetection(int value) {
        arrayDetections[detectionIndex] = value;
        detectionIndex++;
        if (detectionIndex == arrayDetections.length) {
            detectionIndex = 0;
        }
    }

    void drivingCorrectionStraight(double startAngle2, double power) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double difference = imu.getAngularOrientation().firstAngle - startAngle2;
        difference /= Math.PI * 2;
        difference *= power;
        backRight.setPower(power - difference);
        backLeft.setPower(power + difference);
        frontLeft.setPower(power + difference);
        frontRight.setPower(power - difference);
    }

    void drivingCorrectionLeft(double startAngle, double power) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double difference = imu.getAngularOrientation().firstAngle - startAngle;
        difference /= Math.PI * 2;
        difference *= power;
        frontRight.setPower(power - difference);
        frontLeft.setPower(-power + difference);
        backLeft.setPower(power + difference);
        backRight.setPower(-power - difference);
    }

    protected void motorsStop() {
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    boolean tapeSensor45(boolean red) {
        if (red && colorFL.red() > 1500 && colorBR.red() > 500) {
            return false;
        } else return red || colorFR.blue() <= 2000 || colorBL.blue() <= 500;
    }

    boolean tapeSensor90(boolean red) {
        if (red && colorFL.red() > 1500 && colorFR.red() > 3000) {
            return false;
        } else {
            return red || colorFL.blue() <= 2000 || colorFR.blue() <= 2500;
        }
    }

    protected void initAprilTags() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    protected int getAprilTag(double timeOut) {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int lastIDSeen = 0;
        int numofTimesSeen = 0;
        double time = getRuntime();

        while (opModeIsActive() && numofTimesSeen < 10 && (getRuntime() - time) < timeOut) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("lastIDSeen ", lastIDSeen);
            packet.put("numofTimesSeen", numofTimesSeen);

            // If there's been a new frame...
            if (detections != null) {
                // If we don't see any tags
                if (detections.size() == 0) {
                    packet.addLine("detection size = 0");
                    numofTimesSeen = 0;
                    lastIDSeen = 0;
                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    packet.put("id", detections.get(0).id);

                    if (lastIDSeen != detections.get(0).id) {
                        lastIDSeen = detections.get(0).id;
                        numofTimesSeen = 1;
                    } else {
                        numofTimesSeen++;
                    }

                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }
                }

                telemetry.update();
            } else {
                lastIDSeen = 0;
                numofTimesSeen = 0;
                packet.addLine("packet not seen");
            }
            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }

        if (lastIDSeen == 0) {
            lastIDSeen = 2;
        }

        return lastIDSeen;
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

    protected void initialize() {


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        colorFR = hardwareMap.colorSensor.get("colorFR");
        colorFL = hardwareMap.colorSensor.get("colorFL");
        colorBR = hardwareMap.colorSensor.get("colorBR");
        colorBL = hardwareMap.colorSensor.get("colorBL");

        grabbaServo = hardwareMap.servo.get("grabbaServo");
        armuptouch = hardwareMap.touchSensor.get("armuptouch");
        wristServo = hardwareMap.servo.get("wristServo");

        stringMotor = hardwareMap.get(DcMotor.class, "stringMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        stringpot = hardwareMap.get(AnalogInput.class, "stringpot");
        armpot = hardwareMap.get(AnalogInput.class, "armpot");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stringMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("string pot", stringpot.getVoltage());
        telemetry.addData("armpot", armpot.getVoltage());
        telemetry.update();

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
    }

    // picking up = arm pot bigger
    // negative power to pick up - ARM
    protected void armSync(double targVolt) {
        telemetry.addData("armturn", armpot.getVoltage());
        telemetry.update();
        while (opModeIsActive() && Math.abs(armpot.getVoltage() - targVolt) >= 0.01) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("volt", armpot.getVoltage());
            dashboard.sendTelemetryPacket(packet);
            double power = Math.signum(armpot.getVoltage() - targVolt);

            if (power == 1 && (armuptouch.isPressed())) {
                break;
            } else if (power == -1 && (armpot.getVoltage() >= ARMROTATEMAXVOLT)) {
                break;
            } else {
                if (Math.abs(armpot.getVoltage() - targVolt) < .5) {
                    power *= 0.4;
                } else if (Math.abs(armpot.getVoltage() - targVolt) < 1) {
                    power *= 0.5;
                }
                power = Range.clip(power, -0.7, 0.7);
                armMotor.setPower(power);
            }
        }
        armMotor.setPower(0);
    }


    // negative power to go out
    void stringSync(double targetVolt) {
        telemetry.addData("stringturn", stringpot.getVoltage());
        telemetry.update();
        while (opModeIsActive() && Math.abs(stringpot.getVoltage() - targetVolt) >= 0.01) {
            double power = Math.signum(targetVolt - stringpot.getVoltage());

            if (power == -1 && (stringpot.getVoltage() <= VOLTSSTRINGUP)) {
                break;
            } else if (power == 1 && (stringpot.getVoltage() > VOLTSSTRINGDOWN)) {
                break;
            } else {
                if (Math.abs(stringpot.getVoltage() - targetVolt) < .05) {
                    power *= 0.5;
                } else if (Math.abs(stringpot.getVoltage() - targetVolt) < .1) {
                    power *= 0.6;
                }
                stringMotor.setPower(power);
            }
        }
        if (armpot.getVoltage() > 2) {
            stringMotor.setPower(0);
        } else {
            stringMotor.setPower(-0.1);
        }
    }

    protected void returnHome() {
        boolean stringDone = false;
        boolean armDone = false;
        while (opModeIsActive() && (!stringDone || !armDone)) {
            stringDone = stringAsync(VOLTSSTRINGDOWN);
            armDone = armAsync(ARMVOLTSMID, false, 0.9);
        }
    }

    void stringHome() {
        while (opModeIsActive() && stringpot.getVoltage() <= VOLTSSTRINGDOWN) {
            stringMotor.setPower(0.50);
        }
        stringMotor.setPower(0);
    }

    protected void returnMiddle() {
        boolean stringDone = false;
        boolean armDone = false;
        while (opModeIsActive() && (!stringDone || !armDone)) {
            armDone = armAsync(ARMVOLTSMID, true, 1);
            stringDone = stringAsync(VOLTSSTRINGDOWN);
        }
    }


    void scoreMiddlePole() {
        double time = getRuntime();
        TelemetryPacket packet = new TelemetryPacket();


        // Turn arm to initial drop
        grabbaServo.setPosition(1);
        armSync(1.542);
        sleep(100);

        if (getRuntime() - time > 15) {
            return;
        }

        // Extend
        wristPick();
        stringSync(0.368);
        sleep(200);

        if (getRuntime() - time > 15) {
            return;
        }

        armSync(1.7);
        sleep(700);

        if (getRuntime() - time > 15) {
            return;
        }

        grabbaServo.setPosition(0.3);
        sleep(700);

        if (getRuntime() - time > 15) {
            return;
        }

        armSync(1.550);
        sleep(200);

        if (getRuntime() - time > 15) {
            return;
        }

        returnHome();
        sleep(100);
    }

    protected boolean stringAsync(double targetVolt) {
        double power = Math.signum(targetVolt - stringpot.getVoltage());
        if (Math.abs(stringpot.getVoltage() - targetVolt) <= 0.02) {
            stringNoBackDrive();
            return true;
        }
        if (power == -1 && (stringpot.getVoltage() <= VOLTSSTRINGUP)) {
            stringNoBackDrive();
        } else if (power == 1 && (stringpot.getVoltage() > VOLTSSTRINGDOWN)) {
            stringNoBackDrive();
        } else {
            if (Math.abs(stringpot.getVoltage() - targetVolt) < .05) {
                power *= 0.5;
            } else if (Math.abs(stringpot.getVoltage() - targetVolt) < .1) {
                power *= 0.6;
            }
            power = Range.clip(power, -1, 1);
            stringMotor.setPower(power);
        }
        return false;
    }

    protected boolean armAsync(double targVolt, boolean slowDown, double speed, double startDeclerate, double accelerateConstant, double decelerateConstant) {
        double STARTDECELERATE = startDeclerate;
        double ACCELERATECONSTANT = accelerateConstant;
        double DECELERATECONSTANT = decelerateConstant;
        double power = Math.signum(armpot.getVoltage() - targVolt);
        double curPower = Math.abs(armMotor.getPower());
        if (Math.abs(armpot.getVoltage() - targVolt) <= 0.01) {
            armMotor.setPower(0);
            return true;
        }
        if (power == 1 && (armuptouch.isPressed())) {
            armMotor.setPower(0);
            return true;
        } else if (power == -1 && (armpot.getVoltage() >= ARMROTATEMAXVOLT)) {
            armMotor.setPower(0);
            return true;
        } else {
            if (slowDown) {
                if (Math.abs(targVolt - armpot.getVoltage()) > STARTDECELERATE) {
                    // accelerate
                    if (curPower < speed) {
                        curPower += ACCELERATECONSTANT;
                    }
                } else {
                    // decelerate
                    if (curPower>0) {
                        curPower -= DECELERATECONSTANT;
                    }
                }
            }
            else{
                curPower = speed;
            }
            curPower = Range.clip(curPower, 0.35, speed);
            curPower *= power;
            armMotor.setPower(curPower);
        }
        return false;
    }

    protected boolean armAsync(double targVolt, boolean slowDown, double speed) {
        double STARTDECELERATE = 0.2;
        double ACCELERATECONSTANT = 0.03;
        double DECELERATECONSTANT = 0.02;
        return armAsync(targVolt, slowDown, speed, STARTDECELERATE, ACCELERATECONSTANT, DECELERATECONSTANT);
    }


    void stringNoBackDrive() {
        if (armpot.getVoltage() > 2) {
            stringMotor.setPower(0);
        } else {
            stringMotor.setPower(-0.1);
        }
    }

    void turnRobot(double angle, boolean clockwise) {
        double directionalSpeed = clockwise ? 1 : -1;
        double targetAngle = wrap(imu.getAngularOrientation().firstAngle + angle);
        while (opModeIsActive() && !shouldStopTurning(targetAngle)) {

            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();

            if (Math.abs(imu.getAngularOrientation().firstAngle - targetAngle) < 0.05 * Math.PI) {
                directionalSpeed *= 0.3;
            } else if (Math.abs(imu.getAngularOrientation().firstAngle - targetAngle) < 0.1 * Math.PI) {
                directionalSpeed *= 0.5;
            }

            backLeft.setPower(directionalSpeed);
            frontRight.setPower(-directionalSpeed);
            backRight.setPower(-directionalSpeed);
            frontLeft.setPower(directionalSpeed);

        }

        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

    protected void imuAngle() {
        telemetry.addData("IMU Angle", imu.getAngularOrientation().firstAngle);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("IMU Angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);
    }

    protected void grabbaOpen() {
        grabbaServo.setPosition(1);
    }

    protected void grabbaClose() {
        grabbaServo.setPosition(0.3);
    }

    protected void wristDrop() {
        wristServo.setPosition(0.92);
    }

    protected void wristPick() {
        wristServo.setPosition(0);
    }

//    protected void recoveryMode() {
//
//
//    }





    @Override
    public void runOpMode() {

    }
}

