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
public class SimpleAuto extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override
    public void runOpMode() {
        initialize();
        initAprilTags();
        packet.addLine("id before");
        dashboard.sendTelemetryPacket(packet);

        final double TICKSPERBLOCK = 805;
            // 400 per foot

        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;

        waitForStart();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        if (tag == 1){


            int frontpos = frontRight.getCurrentPosition();
            //Continue moving until robot is on tape
            while (opModeIsActive() && tapeSensor45(true) && frontRight.getCurrentPosition() - frontpos < 5500) {
                drivingCorrectionLeft(startAngle, 0.5);
                //  packet.addLine("Red Color " + color1.red());
            }

            motorsStop();
            sleep(1000);


            drivingCorrectionStraight(startAngle, 0.5);
            packet.addLine("spinny");
            dashboard.sendTelemetryPacket(packet);

            double startTime = getRuntime();
            //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
            // this is so robot is for sure off the tape
            int leftpos = frontLeft.getCurrentPosition();

            while (opModeIsActive() && ((tapeSensor90(true) && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
                drivingCorrectionStraight(startAngle, 0.5);
            }

            motorsStop();
            sleep(1000);

        }
        else if (tag == 2){
            telemetry.addLine("tag is 2!");


            int frontpos = frontRight.getCurrentPosition();
            //Continue moving until robot is on tape
            while (opModeIsActive() && tapeSensor45(true) && (frontRight.getCurrentPosition() - frontpos < 5500)) {
                drivingCorrectionLeft(startAngle, 0.5);

            }

            motorsStop();
            sleep(1000);


            drivingCorrectionStraight(startAngle, 0.5);


            double startTime = getRuntime();
            //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
            // this is so robot is for sure off the tape
            int leftpos = frontLeft.getCurrentPosition();

            while (opModeIsActive() && ((tapeSensor90(true) && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
                drivingCorrectionStraight(startAngle, 0.5);
            }

           motorsStop();
            sleep(1000);

            drivingCorrectionLeft(startAngle, -0.5);
            sleep(1000);

        }
        else if (tag == 3){

            while(opModeIsActive() && colorBR.red() < 400){
                drivingCorrectionLeft(startAngle, -0.5);
            }
            motorsStop();
            sleep(1000);


            while(opModeIsActive() && backRight.getCurrentPosition() < 1.5 * TICKSPERBLOCK){
                drivingCorrectionStraight(startAngle, 0.5);
            }

            motorsStop();
            sleep(1000);
        }

    }

}
