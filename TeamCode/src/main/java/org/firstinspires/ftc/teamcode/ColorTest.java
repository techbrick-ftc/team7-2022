package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp
public class ColorTest extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
        public void runOpMode ()
        {
            initialize();
            waitForStart();

            packet.put("encoder", backRight.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            double startAngle = imu.getAngularOrientation().firstAngle;
            while(opModeIsActive() && tapeSensor90(true)) {
                packet.put("encoder", backRight.getCurrentPosition());
                dashboard.sendTelemetryPacket(packet);
                drivingCorrectionStraight(startAngle, 0.25);
            }

            motorsStop();


            while (opModeIsActive()) {
                telemetry.addData("red FL", colorFL.red());
                telemetry.addData("red BR", colorBR.red());
                telemetry.addData("red FR", colorFR.red());
                telemetry.addData("red BL", colorBL.red());
                telemetry.update();
            }
        }
    }
