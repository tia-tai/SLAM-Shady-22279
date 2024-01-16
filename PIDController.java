package org.firstinspires.ftc.teamcode;

import java.util.Date;
import java.util.Timer;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double integralSum = 0;
    private long time;
    private double lastError;
    private boolean initialized = false;

    private double p;
    private double i;
    private double d;

    public PIDController(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        p = 0;
        i = 0;
        d = 0;
    }

    public double update(double target, double state) {
        double error = target - state;

        if (!initialized) {
            time = System.currentTimeMillis() - 5;
            lastError = error;
            initialized = true;
        }
        double timeDiff = (double)(System.currentTimeMillis() - time) / 1000;
        integralSum += error * timeDiff;

        double derivative = (error - lastError) / timeDiff;

        time = System.currentTimeMillis();
        lastError = error;

        p = error * Kp;
        i = integralSum * Ki;
        d = derivative * Kd;

        return p + i + d;
    }

    public double getP() {return p;}
    public double getI() {return i;}
    public double getD() {return d;}
}
