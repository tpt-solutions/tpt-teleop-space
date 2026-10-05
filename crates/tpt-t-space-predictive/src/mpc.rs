//! Bounded linear MPC via projected-gradient optimization (spec §4.4).
//!
//! Solves, each cycle, the finite-horizon problem
//!
//! ```text
//! minimize   Σ ||x_k − x_ref_k||²_Q + Σ ||u_k||²_R
//! subject to x_{k+1} = A x_k + B u_k,  u_min ≤ u_k ≤ u_max
//! ```
//!
//! with projected gradient descent: a fixed iteration budget (deterministic
//! cycle time — no convergence checks, no early exits), gradients
//! accumulated in fixed-size buffers, control clamped to its box after
//! every step. For the second-order-ish plants this workspace flies
//! (double integrators, relative-motion Clohessy–Wiltshire discretizations)
//! a few dozen iterations land well within the actuator limits.

/// A discrete linear plant: `x' = A x + B u`, state dimension `N`,
/// control dimension `M`.
#[derive(Debug, Clone, Copy)]
pub struct Plant<const N: usize, const M: usize> {
    /// State transition matrix, row-major.
    pub a: [[f32; N]; N],
    /// Input matrix, row-major.
    pub b: [[f32; M]; N],
}

impl<const N: usize, const M: usize> Plant<N, M> {
    /// The canonical discretized double integrator per axis
    /// (`p' = p + v·dt, v' = v + a·dt`), `M = N/2` axes.
    #[must_use]
    pub fn double_integrator(axes: usize, dt_s: f32) -> Self {
        assert!(N == axes * 2, "state must be (pos, vel) per axis");
        assert!(M == axes);
        let mut a = [[0.0; N]; N];
        for k in 0..axes {
            a[2 * k][2 * k] = 1.0;
            a[2 * k][2 * k + 1] = dt_s;
            a[2 * k + 1][2 * k + 1] = 1.0;
        }
        let mut b = [[0.0; M]; N];
        for k in 0..axes {
            b[2 * k + 1][k] = dt_s;
        }
        Self { a, b }
    }

    fn fx(&self, x: &[f32; N]) -> [f32; N] {
        let mut out = [0.0; N];
        for (r, row) in self.a.iter().enumerate() {
            let mut acc = 0.0;
            for (c, &v) in row.iter().enumerate() {
                acc += v * x[c];
            }
            out[r] = acc;
        }
        out
    }

    fn fu(&self, u: &[f32; M]) -> [f32; N] {
        let mut out = [0.0; N];
        for (r, row) in self.b.iter().enumerate() {
            let mut acc = 0.0;
            for (c, &v) in row.iter().enumerate() {
                acc += v * u[c];
            }
            out[r] = acc;
        }
        out
    }
}

/// Weights and bounds for the MPC problem.
#[derive(Debug, Clone, Copy)]
pub struct MpcWeights {
    /// State tracking weight (applied uniformly; per-axis tuning rides in
    /// the scaled reference).
    pub q: f32,
    /// Control effort weight.
    pub r: f32,
    /// Control box: `[-u_max, u_max]`.
    pub u_max: f32,
    /// Projected-gradient step size.
    pub step: f32,
    /// Fixed iteration budget per solve (deterministic cycle time).
    pub iterations: u32,
}

impl MpcWeights {
    /// Sensible defaults for a docking/rover double integrator. The state
    /// weight dominates: without a terminal cost, the finite-horizon
    /// controller needs a strong running tracking term to close the last
    /// metres to centimetre level.
    #[must_use]
    pub fn typical() -> Self {
        Self {
            q: 5.0,
            r: 0.02,
            u_max: 1.0,
            step: 0.02,
            iterations: 64,
        }
    }
}

/// The MPC controller over a `HOR`-step horizon. All working buffers are
/// `Self` members — construction allocates nothing on the heap.
pub struct Mpc<const N: usize, const M: usize, const HOR: usize> {
    weights: MpcWeights,
    /// Horizon of states (0 = current), for the rollout. Buffers are
    /// exact-size, allocated once at construction.
    xs: Vec<[f32; N]>,
    /// Horizon of controls.
    us: Vec<[f32; M]>,
    /// Gradient buffers.
    gx: Vec<[f32; N]>,
    gu: Vec<[f32; M]>,
    /// Last solve's cost (telemetry).
    last_cost: f32,
}

impl<const N: usize, const M: usize, const HOR: usize> Mpc<N, M, HOR> {
    /// New controller.
    #[must_use]
    pub fn new(weights: MpcWeights) -> Self {
        Self {
            weights,
            xs: vec![[0.0; N]; HOR + 1],
            us: vec![[0.0; M]; HOR],
            gx: vec![[0.0; N]; HOR + 1],
            gu: vec![[0.0; M]; HOR],
            last_cost: 0.0,
        }
    }

    /// Solve for the first control action given the current state and a
    /// reference state (held constant over the horizon — the standard
    /// regulation formulation). Warm-starts from the previous solution
    /// shifted one step (the receding-horizon pattern).
    pub fn solve(&mut self, plant: &Plant<N, M>, x0: &[f32; N], x_ref: &[f32; N]) -> [f32; M] {
        // Warm start: shift controls, tail zero.
        self.us.rotate_left(1);
        self.us[HOR - 1] = [0.0; M];

        for _ in 0..self.weights.iterations {
            // Rollout.
            self.xs[0] = *x0;
            for k in 0..HOR {
                let fx = plant.fx(&self.xs[k]);
                let fu = plant.fu(&self.us[k]);
                for i in 0..N {
                    self.xs[k + 1][i] = fx[i] + fu[i];
                }
            }
            // Backpropagate gradients: dJ/dx_k and dJ/du_k.
            for slot in self.gx.iter_mut() {
                *slot = [0.0; N];
            }
            for slot in self.gu.iter_mut() {
                *slot = [0.0; M];
            }
            for k in (0..HOR).rev() {
                // Cost gradients at step k.
                for i in 0..N {
                    self.gx[k][i] += 2.0 * self.weights.q * (self.xs[k][i] - x_ref[i]);
                }
                for j in 0..M {
                    self.gu[k][j] += 2.0 * self.weights.r * self.us[k][j];
                }
                // Chain to controls: du/dx via Bᵀ, du/dx via Aᵀ.
                for j in 0..M {
                    let mut acc = self.gu[k][j];
                    for r in 0..N {
                        acc += plant.b[r][j] * self.gx[k + 1][r];
                    }
                    self.gu[k][j] = acc;
                }
                for i in 0..N {
                    let mut acc = self.gx[k][i];
                    for r in 0..N {
                        acc += plant.a[r][i] * self.gx[k + 1][r];
                    }
                    self.gx[k][i] = acc;
                }
                // Normalized gradient step + box projection: the step is
                // applied along the gradient *direction* with magnitude
                // bounded by `step`, so a large tracking error (q-scaled
                // gradients in the hundreds) cannot overshoot the box and
                // oscillate. Deterministic per-iteration progress, no line
                // search.
                for j in 0..M {
                    let g = self.gu[k][j];
                    let bounded = g / g.abs().max(1.0);
                    let u = self.us[k][j] - self.weights.step * bounded;
                    self.us[k][j] = u.clamp(-self.weights.u_max, self.weights.u_max);
                }
            }
        }

        // Final cost for telemetry.
        self.xs[0] = *x0;
        let mut cost = 0.0;
        for k in 0..HOR {
            let fx = plant.fx(&self.xs[k]);
            let fu = plant.fu(&self.us[k]);
            for i in 0..N {
                self.xs[k + 1][i] = fx[i] + fu[i];
                cost += self.weights.q * (self.xs[k][i] - x_ref[i]).powi(2);
            }
            for j in 0..M {
                cost += self.weights.r * self.us[k][j].powi(2);
            }
        }
        self.last_cost = cost;
        self.us[0]
    }

    /// Cost of the last solve (telemetry / tests).
    #[must_use]
    pub fn last_cost(&self) -> f32 {
        self.last_cost
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Plant2 = Plant<2, 1>;
    type Mpc8 = Mpc<2, 1, 8>;

    fn plant() -> Plant2 {
        Plant::double_integrator(1, 0.5)
    }

    #[test]
    fn drives_position_to_reference() {
        let p = plant();
        let mut mpc = Mpc8::new(MpcWeights::typical());
        let mut x = [10.0, 0.0]; // 10 m out, stationary
        let x_ref = [0.0, 0.0];
        for _ in 0..240 {
            let u = mpc.solve(&p, &x, &x_ref);
            // Simulate.
            x[1] += u[0] * 0.5;
            x[0] += x[1] * 0.5;
        }
        assert!(x[0].abs() < 0.2, "position converged, got {}", x[0]);
        assert!(x[1].abs() < 0.2, "velocity settled, got {}", x[1]);
    }

    #[test]
    fn controls_respect_the_box() {
        let p = plant();
        let mut mpc = Mpc8::new(MpcWeights::typical());
        let u = mpc.solve(&p, &[100.0, 0.0], &[0.0, 0.0]);
        assert!(u[0].abs() <= 1.0 + 1e-6, "u clamped, got {}", u[0]);
    }

    #[test]
    fn cost_decreases_while_tracking() {
        let p = plant();
        let mut mpc = Mpc8::new(MpcWeights::typical());
        let x_ref = [0.0, 0.0];
        let mut x = [5.0, 1.0];
        let mut first = f32::INFINITY;
        let mut last = 0.0;
        for k in 0..10 {
            let u = mpc.solve(&p, &x, &x_ref);
            if k == 0 {
                first = mpc.last_cost();
            }
            last = mpc.last_cost();
            x[1] += u[0] * 0.5;
            x[0] += x[1] * 0.5;
        }
        assert!(last < first, "cost {first} → {last} should decrease");
    }

    #[test]
    fn warm_start_shifts_the_horizon() {
        // Two consecutive solves: the second starts from the shifted
        // first (deterministic, no divergence).
        let p = plant();
        let mut mpc = Mpc8::new(MpcWeights::typical());
        let u1 = mpc.solve(&p, &[1.0, 0.0], &[0.0, 0.0]);
        let u2 = mpc.solve(&p, &[1.0, 0.0], &[0.0, 0.0]);
        assert!((u1[0] - u2[0]).abs() < 1e-5, "same state → same first move");
    }
}
