package org.firstinspires.ftc.teamcode.brainstem.utils.controllers;


import com.qualcomm.robotcore.util.Range;

public class MPCController {

    public enum GravityModel {
        NONE,       // For drivetrains
        CONSTANT,   // For elevators/slides
        COSINE      // For pivoting arms
    }

    // MPC Cost Weights
    private double q; // Penalty for position error
    private double r; // Penalty for velocity error
    private double s; // Penalty for control effort (aggressiveness)

    // MPC Horizon Parameters
    private int N;       // Horizon length (e.g., 5-8 steps)
    private double dt;   // Expected loop time in seconds (e.g., 0.02)

    // Feedforward Parameters
    private double kV;
    private double kG;
    private GravityModel gravityModel;

    // The receding horizon control sequence
    private double[] U;

    /**
     * Constructor: Configure the MPC weights, horizon, and physics models.
     */
    public MPCController(double q, double r, double s, int N, double dt,
                                    double kV, double kG, GravityModel gravityModel) {
        this.q = q;
        this.r = r;
        this.s = s;
        this.N = N;
        this.dt = dt;
        this.kV = kV;
        this.kG = kG;
        this.gravityModel = gravityModel;

        this.U = new double[N]; // Initializes with 0.0
    }

    /**
     * Calculates the optimal motor power using online receding-horizon optimization.
     */
    public double calculate(double currentPos, double targetPos, double currentVel, double targetVel) {

        // ==========================================
        // 1. ONLINE MPC OPTIMIZATION (Feedback)
        // ==========================================

        // Shift previous sequence left (Warm Start for receding horizon)
        for (int i = 0; i < N - 1; i++) {
            U[i] = U[i + 1];
        }
        U[N - 1] = 0.0;

        // Simple Gradient Descent Optimizer
        double learningRate = 0.01;
        int iterations = 3;         // Keep this small (3-5) to guarantee fast FTC loop times
        double delta = 0.001;       // Perturbation for numerical gradient

        for (int iter = 0; iter < iterations; iter++) {
            for (int i = 0; i < N; i++) {

                // Calculate cost if we slightly increase U[i]
                U[i] += delta;
                double costUp = simulateCost(currentPos, targetPos, currentVel, targetVel);

                // Calculate cost if we slightly decrease U[i]
                U[i] -= (2 * delta);
                double costDown = simulateCost(currentPos, targetPos, currentVel, targetVel);

                // Reset U[i] to original value
                U[i] += delta;

                // Compute numerical gradient and update
                double gradient = (costUp - costDown) / (2 * delta);
                U[i] -= learningRate * gradient;

                // Constrain the internal prediction to physically possible outputs (-1 to 1)
                U[i] = Range.clip(U[i], -1.0, 1.0);
            }
        }

        // The MPC feedback is simply the first optimal step in the sequence
        double mpcFeedback = U[0];


        // ==========================================
        // 2. PHYSICS PREDICTION (Feedforward)
        // ==========================================

        double feedforward = kV * targetVel;

        switch (gravityModel) {
            case CONSTANT:
                feedforward += kG;
                break;
            case COSINE:
                // currentPos MUST be in radians for this to work
                feedforward += kG * Math.cos(currentPos);
                break;
            case NONE:
            default:
                break;
        }

        // ==========================================
        // 3. COMBINE & RETURN
        // ==========================================

        double totalOutput = mpcFeedback + feedforward;
        return Range.clip(totalOutput, -1.0, 1.0);
    }

    /**
     * Simulates the system forward N steps and calculates the total quadratic cost J.
     */
    private double simulateCost(double pos, double targetPos, double vel, double targetVel) {
        double cost = 0.0;
        double simPos = pos;
        double simVel = vel;

        for (int k = 0; k < N; k++) {
            // Kinematic Model: x_{k+1} = x_k + v_k * dt
            //                  v_{k+1} = v_k + a_k * dt
            simPos += simVel * dt;
            simVel += U[k] * dt;

            // Calculate errors for this predicted step
            double posError = simPos - targetPos;
            double velError = simVel - targetVel;

            // J = q(x - x_target)^2 + r(v - v_target)^2 + s(a)^2
            cost += (q * posError * posError) +
                    (r * velError * velError) +
                    (s * U[k] * U[k]);
        }

        return cost;
    }
}
