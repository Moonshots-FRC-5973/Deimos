package org.firstinspires.ftc.teamcode.wrappers;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.FileWriter;
import java.io.IOException;

public class PIDController {
    public static final double PROPORTIONAL_CONSTANT = .02;
    public static final double INTEGRAL_CONSTANT = .01;
    public static final double DERIVATIVE_CONSTANT = .01;

    private Telemetry telemetry;

    private ElapsedTime runtime = new ElapsedTime();
    private double integralValue = 0.0d;
    private double lastError = 0.0d;

    private double finalError = 0.0d;
    private String name;

    public PIDController(Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        this.name = name;
    }

    public void resetPID() throws IOException {
        FileWriter writer = null;
        try {
            writer = new FileWriter(String.format("%s/FIRST/%s.log", Environment.getExternalStorageDirectory().getPath(), name));
            writer.append(String.format("%f,%f,%f", integralValue, lastError, finalError));
        } catch (IOException e) {
            throw new RuntimeException(e);
        } finally {
            if(writer != null) {
                writer.close();
            }
            integralValue = 0.0d;
            lastError = 0.0d;
            runtime.reset();
        }
    }

    public double getPIDControlledValue(double current, double target) {
        double error = target - current;
        telemetry.addData("x", error);

        integralValue += error * runtime.seconds();

        double dError = error - lastError;

        double p = PROPORTIONAL_CONSTANT * error;
        double i = INTEGRAL_CONSTANT * integralValue;
        double d = DERIVATIVE_CONSTANT * (dError / runtime.seconds());

        finalError = Math.pow(p, 2) + Math.pow(i, 2) + Math.pow(d, 2);

        telemetry.addData("Vp", p);
        telemetry.addData("Vi", i);
        telemetry.addData("Vd", d);
        telemetry.addData("Et", finalError);

        lastError = error;

        runtime.reset();

        return p + i + d;
    }
}
