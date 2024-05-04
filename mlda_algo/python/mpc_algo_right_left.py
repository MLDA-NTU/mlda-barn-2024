from math import sqrt
import casadi as ca
import rospy
import numpy as np
import time


class NMPC:
    def __init__(self, freq=20, N=20):
        self.COLLISION_DIST = 0.36
        self.SAFE_DISTANCE = 0.5

        self.f = freq  # Controller frequency [Hz]
        self.h = 1 / self.f
        # self.rate = rospy.Rate(self.f)

        self.L = 0.37558  # Distance between 2 wheels
        # self.L = rospy.get_param("/mlda/L")
        # Using rosparam

        # For each wheels
        self.v_max_indiv = 1  # Max velocity [m/s]
        self.v_min_indiv = -1  # Min velocity [m/s]
        self.v_max_total = 1
        self.v_min_total = -1

        self.v_ref = 0.5  # Reference velocity [m/s]

        self.a_max = 1  # Max acceleration [m/s^2]

        self.w_max = 1  # Max angular vel [rad/s]
        self.w_min = -1  # Max angular vel [rad/s]

        self.t_max = 0.051
        self.t_min = 0.049

        self.N = N

        self.opt_states = None
        # Obstacle avoidance
        self.mode = "safe"
        pass

    def setup_param(self, mode):
        self.mode = mode

        self.per_step_constraints = 9
        self.init_value_constraints = 5

        self.init_guess = 0

        if mode == "safe":
            self.weight_velocity_ref = 1
            self.weight_max_velocity = 0
            self.weight_position_error = 1
            self.weight_acceleration = 0
            self.weight_cross_track_error = 0
            self.weight_theta_error = 0
            self.weight_inital_theta_error = 0
            self.weight_time_elastic = 0

            self.final_value_contraints = 0
            self.v_ref = 1

            self.v_max_total = 1
            self.v_min_total = -1
            self.w_min = -0.8
            self.w_max = 0.8
        elif mode == "obs":
            self.weight_velocity_ref = 0.1
            self.weight_max_velocity = 0
            self.weight_position_error = 2
            self.weight_acceleration = 0
            self.weight_cross_track_error = 0
            self.weight_theta_error = 0
            self.weight_inital_theta_error = 0
            self.weight_time_elastic = 1
            self.weight_obs = 0

            self.rate = 10
            self.H = 1 / self.rate
            self.final_value_contraints = 0
            self.v_ref = 0.5

            self.v_max_total = 0.8
            self.v_min_total = -0.8
            self.w_min = -0.8
            self.w_max = 0.8
        elif mode == "careful":
            self.weight_velocity_ref = 0.1
            self.weight_max_velocity = 0
            self.weight_position_error = 2
            self.weight_acceleration = 0
            self.weight_cross_track_error = 0
            self.weight_theta_error = 0
            self.weight_inital_theta_error = 0
            self.weight_time_elastic = 1
            self.weight_obs = 0

            self.rate = 10
            self.H = 1 / self.rate
            self.final_value_contraints = 3
            self.v_ref = 0.3

            self.v_max_total = 0.5
            self.v_min_total = -0.5
            self.w_min = -1
            self.w_max = 1

    def setup(self, rate=10):
        self.h = 1 / rate
        # --- State and control variables ---
        # Variables
        # X = [x0, y0, theta0, vr0, vl0, ar0, al0, t0 (...), xN, yN, thetaN, vrN, vlN, arN, alN,tN]
        self.n = (
            8  # n is "Degree of freedom" in 1 instance. # N is the number of time step
        )
        self.X = ca.MX.sym("X", self.N * self.n)
        # Bounds on variables
        lbx = [
            -np.inf,
            -np.inf,
            -np.inf,
            self.v_min_indiv,
            self.v_min_indiv,
            -self.a_max,
            -self.a_max,
            self.t_min,
        ] * self.N
        self.lbx = np.array(lbx)
        ubx = [
            np.inf,
            np.inf,
            np.inf,
            self.v_max_indiv,
            self.v_max_indiv,
            self.a_max,
            self.a_max,
            self.t_max,
        ] * self.N
        self.ubx = np.array(ubx)

        # --- Constraints ---
        # Vehicle dynamics constraints
        # Position (x, y, theta)
        gx = (
            self.X[0 :: self.n][1:]
            - self.X[0 :: self.n][:-1]
            - 0.5
            * self.h
            * (
                ((self.X[3 :: self.n][1:] + self.X[4 :: self.n][1:]) / 2)
                * np.cos(self.X[2 :: self.n][1:])
                + ((self.X[3 :: self.n][:-1] + self.X[4 :: self.n][:-1]) / 2)
                * np.cos(self.X[2 :: self.n][:-1])
            )
        )
        gy = (
            self.X[1 :: self.n][1:]
            - self.X[1 :: self.n][:-1]
            - 0.5
            * self.h
            * (
                ((self.X[3 :: self.n][1:] + self.X[4 :: self.n][1:]) / 2)
                * np.sin(self.X[2 :: self.n][1:])
                + ((self.X[3 :: self.n][:-1] + self.X[4 :: self.n][:-1]) / 2)
                * np.sin(self.X[2 :: self.n][:-1])
            )
        )
        gtheta = (
            self.X[2 :: self.n][1:]
            - self.X[2 :: self.n][:-1]
            - 0.5
            * self.h
            * (
                ((self.X[3 :: self.n][1:] - self.X[4 :: self.n][1:]) / self.L)
                + ((self.X[3 :: self.n][:-1] - self.X[4 :: self.n][:-1]) / self.L)
            )
        )
        # Velocity (v_r, v_l)
        gv_r = (
            self.X[3 :: self.n][1:]
            - self.X[3 :: self.n][:-1]
            - 0.5 * self.h * (self.X[5 :: self.n][1:] + self.X[5 :: self.n][:-1])
        )
        gv_l = (
            self.X[4 :: self.n][1:]
            - self.X[4 :: self.n][:-1]
            - 0.5
            * self.h
            * (
                self.X[6 :: self.n][1:] + self.X[6 :: self.n][:-1]
            )  # Time is an opt variable
        )
        # Positive linear velocity
        gv_min = (
            self.X[3 :: self.n][1:] + self.X[4 :: self.n][1:]
        ) - self.v_min_total * 2
        gv_max = self.v_max_total * 2 - (
            self.X[3 :: self.n][1:] + self.X[4 :: self.n][1:]
        )

        # Minimum angular velocity
        gw_min = (
            (self.X[3 :: self.n][1:] - self.X[4 :: self.n][1:]) / self.L
        ) - self.w_min
        # Maximum angular velocity
        gw_max = self.w_max - (
            (self.X[3 :: self.n][1:] - self.X[4 :: self.n][1:]) / self.L
        )

        ### Each of the term above has N-1 columns
        self.g = ca.vertcat(gx, gy, gtheta, gv_r, gv_l, gv_min, gv_max, gw_min, gw_max)

        ### This is not dynamic contraints, because they only relate the existing optimization variables

    def solve(self, x_ref, y_ref, theta_ref, X0):
        self.setup_param("safe")
        self.setup(10)
        start_time = time.time()

        # === Set constraints bound
        self.lbg = np.zeros(
            (self.N - 1) * self.per_step_constraints
            + self.init_value_constraints
            + self.final_value_contraints
        )
        self.ubg = np.zeros(
            (self.N - 1) * self.per_step_constraints
            + self.init_value_constraints
            + self.final_value_contraints
        )
        # Set inequality bound
        self.ubg[
            (
                -4 * (self.N - 1)
                - self.init_value_constraints
                - self.final_value_contraints
            ) : (-self.init_value_constraints - self.final_value_contraints)
        ] = np.inf  # Velocity positive

        ## All constraints g
        self.g = self.g[
            : (self.N - 1) * self.per_step_constraints
        ]  # Since we are recycling the same instance

        # Init value constraints expression
        for i in range(self.init_value_constraints):
            g0 = self.X[i :: self.n][0] - X0[i]
            self.g = ca.vertcat(self.g, g0)

        def diff_angle(a1, a2):
            diff = max(a1, a2) - min(a1, a2)
            if diff > np.pi:
                diff = 2 * np.pi - diff
            return diff

        ref = theta_ref[self.N]
        idx = 0
        for i in range(self.N, min(len(x_ref), len(theta_ref))):
            # print(i, " Diff angle: ", diff_angle(theta_ref[i],ref))
            idx = i
            if diff_angle(theta_ref[i], ref) > np.pi / 25:
                break

        # idx = self.N - 1
        gfx = self.X[0 :: self.n][self.N - 1] - x_ref[idx]
        gfy = self.X[1 :: self.n][self.N - 1] - y_ref[idx]
        gftheta = self.X[2 :: self.n][self.N - 1] - theta_ref[idx]
        if self.final_value_contraints != 0:
            self.g = ca.vertcat(self.g, gfx, gfy, gftheta)

        # --- Cost function ---

        J = 0
        initial_theta_error = (self.X[2 :: self.n][1] - theta_ref[1]) ** 2
        J += self.weight_inital_theta_error * initial_theta_error

        for i in range(self.N):
            # Position Error cost
            position_error_cost = (self.X[0 :: self.n][i] - x_ref[i]) ** 2 + (
                self.X[1 :: self.n][i] - y_ref[i]
            ) ** 2

            # Theta Error cost
            theta_error_cost = (self.X[2 :: self.n][i] - theta_ref[i]) ** 2

            # Reference velocity cost
            reference_velocity_cost = ((self.X[3 :: self.n][i]) - self.v_ref) ** 2 + (
                (self.X[4 :: self.n][i]) - self.v_ref
            ) ** 2
            # reference_velocity_cost = 0

            # Cross-track Error cost
            cross_track_error_cost = -(x_ref[i] - self.X[0 :: self.n][i]) * (
                np.sin(theta_ref[i])
            ) + (y_ref[i] - self.X[1 :: self.n][i]) * (np.cos(theta_ref[i]))

            maximize_velocity = -(
                self.X[3 :: self.n][i] ** 2 + self.X[4 :: self.n][i] ** 2
            )

            # Successive control cost
            if i != (self.N - 1):
                successive_error = (
                    (self.X[5 :: self.n][i + 1] - self.X[5 :: self.n][i])
                    * (self.X[5 :: self.n][i + 1] - self.X[5 :: self.n][i])
                ) + (
                    (self.X[6 :: self.n][i + 1] - self.X[6 :: self.n][i])
                    * (self.X[6 :: self.n][i + 1] - self.X[6 :: self.n][i])
                )

            # Cost function calculation
            J += (
                self.weight_position_error * position_error_cost
                + self.weight_velocity_ref * reference_velocity_cost
                + self.weight_theta_error * theta_error_cost
                + self.weight_cross_track_error * cross_track_error_cost
                + self.weight_acceleration * successive_error
                + self.weight_max_velocity * maximize_velocity
            )

        # === Initial guess
        self.init_guess = ca.GenDM_zeros(self.N * self.n, 1)
        self.init_guess[:: self.n] = x_ref[: self.N]
        self.init_guess[1 :: self.n] = y_ref[: self.N]
        self.init_guess[2 :: self.n] = theta_ref[: self.N]
        init_guess = self.init_guess

        # === Solution
        options = {
            "ipopt.print_level": 1,
            "print_time": 0,
            "expand": 1,
        }  # Dictionary of the options
        # options = {}

        problem = {"f": J, "x": self.X, "g": self.g}  # Dictionary of the problem
        solver = ca.nlpsol(
            "solver", "ipopt", problem, options
        )  # ipopt: interior point method

        solution = solver(
            x0=init_guess, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg
        )

        # === Optimal control
        vr_opt = solution["x"][3 :: self.n][1]
        vl_opt = solution["x"][4 :: self.n][1]

        v_opt = (vr_opt + vl_opt) / 2
        w_opt = (vr_opt - vl_opt) / self.L

        vr_opt_list = solution["x"][3 :: self.n]
        vl_opt_list = solution["x"][4 :: self.n]
        v_opt_list = (vr_opt_list + vl_opt_list) / 2
        w_opt_list = (vr_opt_list - vl_opt_list) / self.L

        # Intial guess for next steps
        self.opt_states = solution["x"]
        solve_time = round(time.time() - start_time, 5)

        print(
            "V:",
            np.round(np.array(v_opt).item(), 3),
            " W:",
            np.round(np.array(w_opt).item(), 3),
            "Rate:",
            round(1 / solve_time, 1),
            "Cost:",
            np.round(np.array(solution["f"]).item(), 1),
            "=={}=={}==".format(self.v_ref, self.v_max_indiv),
        )

        return v_opt, w_opt, self.mode, solve_time

    def solve_obs(self, x_ref, y_ref, theta_ref, obs_x, obs_y, X0):
        start_time = time.time()

        obs_num = len(obs_x)
        obs_constraints = obs_num * (self.N - 1)

        obs_dist = []
        for i in range(obs_num):
            obs_dist.append(
                np.sqrt((obs_x[i] - x_ref[0]) ** 2 + (obs_y[i] - y_ref[0]) ** 2)
            )
        obs_dist = np.array(obs_dist)

        careful = False
        if np.count_nonzero(obs_dist < self.SAFE_DISTANCE) > 0:  # m
            self.setup_param("careful")

        else:
            self.setup_param("obs")
        self.setup(10)

        # === Set constraints bound

        # final_value_contraints = 0

        self.lbg = np.zeros(
            (self.N - 1) * self.per_step_constraints
            + self.init_value_constraints
            + self.final_value_contraints
            + obs_constraints
        )
        self.ubg = np.zeros(
            (self.N - 1) * self.per_step_constraints
            + self.init_value_constraints
            + self.final_value_contraints
            + obs_constraints
        )
        # Set inequality bound
        self.ubg[
            (
                -4 * (self.N - 1)
                - self.init_value_constraints
                - self.final_value_contraints
                - obs_constraints
            ) : (
                -self.init_value_constraints
                - self.final_value_contraints
                - obs_constraints
            )
        ] = np.inf  # Velocity positive

        self.ubg[(-obs_constraints):] = np.inf  # Obstacle avoidance

        ## All constraints g
        self.g = self.g[
            : (self.N - 1) * self.per_step_constraints
        ]  # Since we are recycling the same instance
        # We have to truncate the previous optimization run

        # Init value constraints expression
        for i in range(self.init_value_constraints):
            g0 = self.X[i :: self.n][0] - X0[i]
            self.g = ca.vertcat(self.g, g0)

        # Final value constraints expression

        offset = self.N - 1
        gfx = self.X[0 :: self.n][offset] - x_ref[offset]
        gfy = self.X[1 :: self.n][offset] - y_ref[offset]
        gftheta = self.X[2 :: self.n][offset] - theta_ref[offset]
        if self.final_value_contraints != 0:
            self.g = ca.vertcat(self.g, gfx, gfy, gftheta)

        for i in range(obs_num):
            gobs = (
                (obs_x[i] - self.X[0 :: self.n][1:]) ** 2
                + (obs_y[i] - self.X[1 :: self.n][1:]) ** 2
                - self.COLLISION_DIST**2
            )
            self.g = ca.vertcat(self.g, gobs)

        # --- Cost function ---

        J = 0

        # initial_theta_error = (self.X[2 :: self.n][1] - theta_ref[1]) ** 2
        # J += self.weight_inital_theta_error * initial_theta_error

        # for i in range(obs_num):

        #     for j in range(self.N - 1):
        #         obs_dist = ca.fmax(
        #             self.COLLISION_DIST**2
        #             - (obs_x[i] - self.X[0 :: self.n][j]) ** 2
        #             - (obs_y[i] - self.X[1 :: self.n][j]) ** 2,
        #             ca.MX(0),
        #         )
        #         J += self.weight_obs * obs_dist

        for i in range(self.N):
            # Position Error cost
            position_error_cost = (self.X[0 :: self.n][i] - x_ref[i]) ** 2 + (
                self.X[1 :: self.n][i] - y_ref[i]
            ) ** 2

            # Theta Error cost
            theta_error_cost = (self.X[2 :: self.n][i] - theta_ref[i]) ** 2

            # Reference velocity cost
            reference_velocity_cost = ((self.X[3 :: self.n][i]) - self.v_ref) ** 2 + (
                (self.X[4 :: self.n][i]) - self.v_ref
            ) ** 2
            maximize_velocity = -(
                self.X[3 :: self.n][i] ** 2 + self.X[4 :: self.n][i] ** 2
            )
            # Cross-track Error cost
            cross_track_error_cost = -(x_ref[i] - self.X[0 :: self.n][i]) * (
                np.sin(theta_ref[i])
            ) + (y_ref[i] - self.X[1 :: self.n][i]) * (np.cos(theta_ref[i]))

            # Successive control cost
            if i != (self.N - 1):
                successive_error = (
                    (self.X[5 :: self.n][i + 1] - self.X[5 :: self.n][i])
                    * (self.X[5 :: self.n][i + 1] - self.X[5 :: self.n][i])
                ) + (
                    (self.X[6 :: self.n][i + 1] - self.X[6 :: self.n][i])
                    * (self.X[6 :: self.n][i + 1] - self.X[6 :: self.n][i])
                )

            time_elastic = (self.X[7 :: self.n][i] - self.H) ** 2
            # Cost function calculation
            J += (
                self.weight_position_error * position_error_cost
                + self.weight_velocity_ref * reference_velocity_cost
                + self.weight_theta_error * theta_error_cost
                + self.weight_cross_track_error * cross_track_error_cost
                + self.weight_acceleration * successive_error
                + self.weight_max_velocity * maximize_velocity
                + self.weight_time_elastic * time_elastic
            )

        # === Initial guess
        self.init_guess = ca.GenDM_zeros(self.N * self.n, 1)
        self.init_guess[:: self.n] = x_ref[: self.N]
        self.init_guess[1 :: self.n] = y_ref[: self.N]
        self.init_guess[2 :: self.n] = theta_ref[: self.N]
        init_guess = self.init_guess
        # === Solution
        options = {
            "ipopt.print_level": 1,
            "print_time": 0,
            "expand": 1,
        }  # Dictionary of the options
        # options = {}

        problem = {"f": J, "x": self.X, "g": self.g}  # Dictionary of the problem
        solver = ca.nlpsol(
            "solver", "ipopt", problem, options
        )  # ipopt: interior point method

        solution = solver(
            x0=init_guess, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg
        )

        # === Optimal control
        vr_opt = solution["x"][3 :: self.n][1]
        vl_opt = solution["x"][4 :: self.n][1]

        v_opt = (vr_opt + vl_opt) / 2
        w_opt = (vr_opt - vl_opt) / self.L

        vr_opt_list = solution["x"][3 :: self.n]
        vl_opt_list = solution["x"][4 :: self.n]
        v_opt_list = (vr_opt_list + vl_opt_list) / 2
        w_opt_list = (vr_opt_list - vl_opt_list) / self.L

        # Intial guess for next steps
        self.opt_states = solution["x"]
        solve_time = round(time.time() - start_time, 5)

        print(
            "V:",
            np.round(np.array(v_opt).item(), 3),
            " W:",
            np.round(np.array(w_opt).item(), 3),
            "Rate:",
            round(1 / solve_time, 1),
            "Cost:",
            np.round(np.array(solution["f"]).item(), 1),
            "=={}=={}==".format(self.v_ref, self.mode),
        )

        return v_opt, w_opt, self.mode, solve_time
