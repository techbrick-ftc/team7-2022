package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp

public class PotenTest extends StarterAuto {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode()  {

        initialize();
        waitForStart();

        armpotTurn(90);
        sleep(1000);

        stringpotTurn(50);
        sleep(1000);

        while(opModeIsActive()) {

            double currentVolt = stringpot.getVoltage();

            packet.put("volt", currentVolt);
            dashboard.sendTelemetryPacket(packet);
        }






    }
}
