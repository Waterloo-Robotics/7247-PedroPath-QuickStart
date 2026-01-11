package org.firstinspires.ftc.teamcode.Modules;

public class AngleModule {
    public PIDController pid_controller;

    public AngleModule()
    {
        this.pid_controller = new PIDController((float) 0.012, (float)0.05, 0);
        this.pid_controller.set_input_limits(-180, 180, true);
        this.pid_controller.set_output_limits(-0.6, 0.6);
        this.pid_controller.set_acceleration_limit(0.025);
    }

    public double set_Angle( double angle, double current_angle)
    {

        // Update PID Controller
        this.pid_controller.set_target((float)angle);
        return this.pid_controller.update((float)current_angle);
    }
}


