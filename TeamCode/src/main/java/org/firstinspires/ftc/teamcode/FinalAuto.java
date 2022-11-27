package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Autonomous
public class FinalAuto extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        initialize();
        initAprilTags();

        final double TICKSPERBLOCK = 805;
        // 400 per foot

        double startAngle = imu.getAngularOrientation().firstAngle;

        waitForStart();
        int tag = getAprilTag(5);

        int frontpos = frontRight.getCurrentPosition();
        //Continue moving until robot is on diagonal red tape
        while (opModeIsActive() && tapeSensor45(true) && frontRight.getCurrentPosition() - frontpos < 5500) {
            drivingCorrectionLeft(startAngle, 0.35);
        }

        motorsStop();
        sleep(1000);

        // loop until strafe you strafe a little right, want to make sure you dont hit stack of cups

        startAngle = imu.getAngularOrientation().firstAngle;
        int backleftpos = backLeft.getCurrentPosition();
        while(opModeIsActive() && backLeft.getCurrentPosition() - backleftpos < TICKSPERBLOCK * 0.05){
            drivingCorrectionLeft(startAngle, -0.5);
        }
        motorsStop();
        sleep(500);

        drivingCorrectionStraight(startAngle, 0.5);


        double startTime = getRuntime();
        //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
        // this is so robot is for sure off the tape
        int frontleftpos = frontLeft.getCurrentPosition();

        packet.addLine("before 90");
        dashboard.sendTelemetryPacket(packet);

        // Go moving until robot is on straight red tape
        while (opModeIsActive() && ((tapeSensor90(true) && frontLeft.getCurrentPosition() - frontleftpos < 4500) || getRuntime() - startTime < 1)) {
            drivingCorrectionStraight(startAngle, 0.5);
        }
        motorsStop();

        packet.addLine("after 90");
        dashboard.sendTelemetryPacket(packet);

        sleep(500);

        turnRobot(-Math.PI / 2,true);
        startAngle = imu.getAngularOrientation().firstAngle;

        packet.addLine("after turn");
        dashboard.sendTelemetryPacket(packet);

        int backrightpos = backRight.getCurrentPosition();




        // park
        if (tag == 1){
            motorsStop();
        }

        // Go straight until in spot #2
        else if (tag == 2){ 
            while (opModeIsActive() && backRight.getCurrentPosition() - backrightpos < 0.6 * TICKSPERBLOCK){
                drivingCorrectionStraight(startAngle, 0.5);
            }
            motorsStop();
        }

        // Go straight until in spot #3
        else if (tag == 3){
            while (opModeIsActive() && backRight.getCurrentPosition() - backrightpos < 1.7 * TICKSPERBLOCK){
                drivingCorrectionStraight(startAngle, 0.5);
            }
            motorsStop();
        }
    }
}
