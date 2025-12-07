package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class flywheelModule {

    public DcMotorEx flywheel;
    public double output_power;
    public double feedforward_power;
    public double pid_power;
    public double motor_speed_rpm;
    public PIDController pid_controller;

    public flywheelModule(DcMotor dcMotor)
    {
        this.flywheel = (DcMotorEx)dcMotor;
        this.pid_controller = new PIDController((float)0.002, 0, 0);
    }

    public void set_speed(int speed_rpm)
    {
        /* Calculate Feedforward
        * the base power to use based on the maximum rpm the motor spins
        * at 100% power. Then limit between -1 and 1. */
        double calc_power = ((float)speed_rpm) / 4100;
        double limit_1 = Math.min(calc_power, 1);
        this.feedforward_power = Math.max(limit_1, -1);

        /* Calculate PID Power
        * Use the motor encoder and a PID controller to add or subtract power
        * from the feedforward to the speed closer */

        // Get the current motor speed and convert to RPM
        this.motor_speed_rpm = this.flywheel.getVelocity() / 28.0 * 60.0;

        // Update PID Controller
        this.pid_controller.set_target(speed_rpm);
        this.pid_power = this.pid_controller.update(this.motor_speed_rpm);

        this.output_power = this.feedforward_power + this.pid_power;
        this.flywheel.setPower(this.output_power);
    }
}


