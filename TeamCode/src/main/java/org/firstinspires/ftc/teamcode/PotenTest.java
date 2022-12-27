package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

@Disabled
@TeleOp

public class PotenTest extends StarterAuto {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode()  {

        initialize();
        waitForStart();

        armSync(90);
        sleep(1000);

        stringSync(50);
        sleep(1000);

        while(opModeIsActive()) {

            double currentVolt = stringpot.getVoltage();

            packet.put("volt", currentVolt);
            dashboard.sendTelemetryPacket(packet);
        }






    }
}
