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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
/*
Configurations:
Expansion Hub:
 I2C Port 0: colorFR     aka color1
 I2C Port 1: colorFL
 I2C Port 2: colorBL
 I2C Port 3: colorBR

 Servo Port 0: grabbaServo
 Servo Port 5: wristServo

 Motor Port 0: armMotor
 Motor Port 2: stringMotor

 Control Hub:
  Motor Port 0: frontRight
  Motor Port 1: frontLeft
  Motor Port 2: backLeft
  Motor Port 3: backRight

  I2C Port 0: imu

  Digital 0: armuptouch
 */

public class StarterAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 1431.704;
    double fy = 1437.693;
    double cx = 641.105;
    double cy = 353.960;

    // UNITS ARE METERS
    double tagsize = 0.045;


    int numFramesWithoutDetection = 0;

    int[] arrayDetections = new int[64];
    int detectionIndex = 0;


    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public void insertDetection(int value) {
        arrayDetections[detectionIndex] = value;
        detectionIndex++;
        if (detectionIndex == arrayDetections.length) {
            detectionIndex = 0;
        }
    }

    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public BNO055IMU imu;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public ColorSensor colorFR;
    public ColorSensor colorFL;
    public ColorSensor colorBR;
    public ColorSensor colorBL;
    public TouchSensor armuptouch;
    public Servo grabbaServo;
    public Servo wristServo;

    void drivingCorrectionStraight(double startAngle2, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle2;
        backRight.setPower(power + difference);
        frontLeft.setPower(power - difference);
    }

    void drivingCorrectionLeft(double startAngle, double power) {
        double difference = imu.getAngularOrientation().firstAngle - startAngle;
        frontRight.setPower(power + difference);
        backLeft.setPower(power - difference);
    }

    void initAprilTags() {
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

    int getAprilTag() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int lastIDSeen = 0;
        int numofTimesSeen = 0;
        while (opModeIsActive() && numofTimesSeen < 10){
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("lastIDSeen " , lastIDSeen);
            packet.put("numofTimesSeen", numofTimesSeen);

            // If there's been a new frame...
            if(detections != null)
            {
                // If we don't see any tags
                if(detections.size() == 0)
                {
                    packet.addLine("detection size = 0");
                    numofTimesSeen = 0;
                    lastIDSeen = 0;
                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    packet.put("id", detections.get(0).id);

                    if (lastIDSeen != detections.get(0).id) {
                        lastIDSeen = detections.get(0).id;
                        numofTimesSeen = 1;
                    }
                    else {
                        numofTimesSeen ++;
                    }

                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                }

                telemetry.update();
            }
            else {
                lastIDSeen = 0;
                numofTimesSeen = 0;
                packet.addLine("packet not seen");
            }
            dashboard.sendTelemetryPacket(packet);
            sleep(20);

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

    void initialize(){
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

    }



    void turnRobot(double angle, double speed, boolean clockwise) {
        double directionalSpeed = clockwise ? -speed : speed;
        double targetAngle = wrap(imu.getAngularOrientation().firstAngle + angle);
        while (opModeIsActive() && !shouldStopTurning(targetAngle)) {
            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();
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

    @Override
    public void runOpMode() {

    }
}
