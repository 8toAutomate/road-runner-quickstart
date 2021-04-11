package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@Autonomous(name="CycleTest", group="Motion")

public class CycleTest extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 21.1;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        waitForStart();

        long setTime = System.currentTimeMillis();

        for (int i=0; i<10000000; i++) {}

        long diff = System.currentTimeMillis() - setTime;

        telemetry.addData("time taken: ", diff);
        telemetry.update();

        while (opModeIsActive()) {}
    }

}
