package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    public double target;
    public double current;
    public double error;
    public double output;
    private double last_error;

    /* PID Constants */
    private float kP;
    private float kI;
    private float kD;

    /* Output Terms */
    private double pTerm;
    private double iTerm;
    private double dTerm;

    /* Configuration */
    private double tolerance;
    private double max_output = 1;
    private double min_output = -1;
    private double max_input = 1;
    private double min_input = -1;
    private boolean continuous_input = false;
    private double max_acceleration = 0;
    private boolean use_max_acceleration = false;

    /* Locals */
    private boolean limited;

    public PIDController(float p, float i, float d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void set_target(float target) {
        this.target = target;
    }

    public void set_tolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double update(double current_position) {
        if (this.continuous_input) {
            /* Target is -10 and we're at 180 = -190, we want 170 */
            this.error = this.target - current_position;

            if (this.error < this.min_input) {this.error = this.error + (this.max_input - this.min_input);}
            if (this.error > this.max_input) {this.error = this.error - (this.max_input - this.min_input);}
        }
        else {
            this.error = this.target - current_position;
        }

        this.pTerm = this.error * this.kP;

        /* Only add to the integrator if we're not saturated */
        if (!this.limited && Math.abs(this.error) < 10) {
            this.iTerm = iTerm + (this.error * this.kI);
        }

        this.dTerm = (this.error - this.last_error) * this.kD;

        double raw_output = this.pTerm + this.iTerm + this.dTerm;
        double bound_output;

        /* Bound raw output with min and max output */
        if (raw_output < this.min_output) {
            this.limited = true;
            bound_output = this.min_output;
        } else if (raw_output > this.max_output) {
            this.limited = true;
            bound_output = this.max_output;
        } else {
            this.limited = false;
            bound_output = raw_output;
        }

        /* Limit the acceleration */
        if (this.use_max_acceleration && (Math.abs(bound_output) > Math.abs(this.output)) && (Math.abs(bound_output - this.output) > this.max_acceleration))
        {
            this.output = this.output + (this.max_acceleration * Math.signum(bound_output - this.output));
        }
        else{
            this.output = bound_output;
        }

        return this.output;
    }

    public boolean at_target() {
        return (Math.abs(this.error) < this.tolerance);
    }

    public void set_output_limits(double min, double max) {
        this.min_output = min;
        this.max_output = max;
    }

    public void set_input_limits(double min, double max, boolean continuous) {
        this.min_input = min;
        this.max_input = max;
        this.continuous_input = continuous;
    }

    public void set_acceleration_limit(double limit) {
        this.max_acceleration = limit;
        this.use_max_acceleration = true;
    }

    public void add_telemetry(Telemetry telemetry)
    {
        telemetry.addData("P Term", this.pTerm);
        telemetry.addData("I Term", this.iTerm);
        telemetry.addData("D Term", this.dTerm);
    }

}
