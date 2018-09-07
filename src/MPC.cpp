#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

namespace
{
    const double MAX_SPEED       = 100.0;
    const unsigned int N         = 10;
    const double dt              = 0.05;
    const double CURVATURE_SCALE = 25.0;

    // The length from front to CoG that has a similar radius.
    const double Lf = 2.67;

    //Start indexes for states
    const unsigned int x_start     = 0;
    const unsigned int y_start     = x_start     + N;
    const unsigned int psi_start   = y_start     + N;
    const unsigned int v_start     = psi_start   + N;
    const unsigned int cte_start   = v_start     + N;
    const unsigned int epsi_start  = cte_start   + N;
    const unsigned int delta_start = epsi_start  + N;
    const unsigned int a_start     = delta_start + N - 1;

    //Cubic polynomial calculation
    AD<double> Poly3Eval(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        AD<double> x2 = x * x;
        AD<double> x3 = x2 * x;

        return (coeffs[0] + coeffs[1] * x + coeffs[2] * x2 + coeffs[3] * x3);
    }

    //First derivative of cubic polynomial
    AD<double> Poly3Dot1Eval(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        return (coeffs[1] + 2.0 * x * coeffs[2] + 3.0 * x * x * coeffs[3]);
    }

    //Second derivative of cubic polynomial
    AD<double> Poly3Dot2Eval(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        return (2.0 * coeffs[2] + 6.0 * x * coeffs[3]);
    }

    //Curvature calculation
    AD<double> Curvature(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        AD<double> d2 = Poly3Dot2Eval(coeffs, x);
        AD<double> d1 = Poly3Dot1Eval(coeffs, x);

        return ( d2 / pow( (1.0 + d1 * d1), 1.5) );
    }

    double Rad2Deg(double rad)
    {
        return (rad * 180.0 / acos(-1.0));
    }
}

class FG_eval
{
    public:

        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;

        FG_eval(Eigen::VectorXd coeffs)
        {
            this->coeffs = coeffs;
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        void operator()(ADvector& fg, const ADvector& vars)
        {
            //Curvature adaptation for reference speed
            AD<double> start_curvature = Curvature(coeffs, vars[x_start]);
            AD<double> reference_speed = MAX_SPEED / (1.0 + CURVATURE_SCALE * fabs(start_curvature));

            std::cout << "Start curvature: "   << start_curvature
                      << ", Maximum speed: "   << MAX_SPEED
                      << ", Curvature scale: " << CURVATURE_SCALE
                      << ", Reference speed: " << reference_speed
                      << std::endl;

            //Cost
            fg[0] = 0;

            // The part of the cost based on the reference state.
            for(unsigned int t = 0; t < N; ++t)
            {
                fg[0] += CppAD::pow(vars[cte_start  + t], 2);
                fg[0] += CppAD::pow(vars[epsi_start + t], 2);
                fg[0] += CppAD::pow((vars[v_start   + t] - reference_speed), 2);
            }

            // Minimize the use of actuators.
            for(unsigned int t = 0; t < (N - 1); ++t)
            {
                fg[0] += CppAD::pow(vars[delta_start + t], 2);
                fg[0] += CppAD::pow(vars[a_start     + t], 2);
            }

            // Minimize the value gap between sequential actuations.
            for(unsigned int t = 0; t < (N - 2); ++t)
            {
                fg[0] += (reference_speed * reference_speed) * CppAD::pow((vars[delta_start + t + 1] - vars[delta_start + t]), 2);
                fg[0] += CppAD::pow((vars[a_start + t + 1] - vars[a_start + t]), 2);
            }

            // Setup Constraints
            fg[1 + x_start]    = vars[x_start];
            fg[1 + y_start]    = vars[y_start];
            fg[1 + psi_start]  = vars[psi_start];
            fg[1 + v_start]    = vars[v_start];
            fg[1 + cte_start]  = vars[cte_start];
            fg[1 + epsi_start] = vars[epsi_start];

            // The rest of the constraints
            for(unsigned int t = 1; t < N; ++t)
            {
                // The state at time t+1 .
                AD<double> x1      = vars[x_start    + t];
                AD<double> y1      = vars[y_start    + t];
                AD<double> psi1    = vars[psi_start  + t];
                AD<double> v1      = vars[v_start    + t];
                AD<double> cte1    = vars[cte_start  + t];
                AD<double> epsi1   = vars[epsi_start + t];

                // The state at time t.
                AD<double> x0      = vars[x_start    + t - 1];
                AD<double> y0      = vars[y_start    + t - 1];
                AD<double> psi0    = vars[psi_start  + t - 1];
                AD<double> v0      = vars[v_start    + t - 1];
                AD<double> cte0    = vars[cte_start  + t - 1];
                AD<double> epsi0   = vars[epsi_start + t - 1];

                // Only consider the actuation at time t.
                AD<double> delta0  = vars[delta_start + t - 1];
                AD<double> a0      = vars[a_start     + t - 1];

                AD<double> f0      = Poly3Eval(coeffs, x0);
                AD<double> psides0 = CppAD::atan(Poly3Dot1Eval(coeffs, x0));

                //The idea here is to constraint this value to be 0.
                //The equations for the model:
                // x_[t+1]   = x[t]   + v[t] * cos(psi[t]) * dt
                // y_[t+1]   = y[t]   + v[t] * sin(psi[t]) * dt
                // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
                // v_[t+1]   = v[t]   + a[t] * dt
                // cte[t+1]  = y[t]   - f(x[t]) + v[t] * sin(epsi[t]) * dt
                // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

                fg[1 + x_start    + t] = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
                fg[1 + y_start    + t] = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
                fg[1 + psi_start  + t] = psi1  - (psi0 + v0 * delta0 / Lf * dt);
                fg[1 + v_start    + t] = v1    - (v0 + a0 * dt);
                fg[1 + cte_start  + t] = cte1  - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
                fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
            }
        }
};

//
// MPC class definition implementation.
//
MPC::MPC()
:m_delta(0.0),
 m_a(0.0),
 m_solution_x(0),
 m_solution_y(0)
{
}

MPC::~MPC() {}

vector<double> MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, double delay)
{
    typedef CPPAD_TESTVECTOR(double) Dvector;

    std::cout << "Vehicle State";
    for(unsigned int i = 0; i < state.size(); ++i)
    {
        std::cout << ", " << state[i];
    }
    std::cout << std::endl;

    Eigen::VectorXd proj_state = ProjectState(state, coeffs, delay);

    std::cout << "Projected State";
    for(unsigned int i = 0; i < proj_state.size(); ++i)
    {
        std::cout << ", " << proj_state[i];
    }
    std::cout << std::endl;

    double x    = proj_state[0];
    double y    = proj_state[1];
    double psi  = proj_state[2];
    double v    = proj_state[3];
    double cte  = proj_state[4];
    double epsi = proj_state[5];

    //Number of model varialbles including states and inputs
    unsigned int n_vars = N * 6 + (N - 1) * 2;

    //Set the number of constraints
    unsigned int n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for(unsigned int i = 0; i < n_vars; ++i)
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start]    = x;
    vars[y_start]    = y;
    vars[psi_start]  = psi;
    vars[v_start]    = v;
    vars[cte_start]  = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    //Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for(unsigned int i = 0; i < delta_start; ++i)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] =  1.0e19;
    }

    // The upper and lower limits of delta (steering angle are set
    // to -25 and 25 degrees (values in radians).
    for(unsigned int i = delta_start; i < a_start; ++i)

    {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] =  0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for(unsigned int i = a_start; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -2.0;
        vars_upperbound[i] =  2.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (unsigned int i = 0; i < n_constraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start]    = x;
    constraints_lowerbound[y_start]    = y;
    constraints_lowerbound[psi_start]  = psi;
    constraints_lowerbound[v_start]    = v;
    constraints_lowerbound[cte_start]  = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start]    = x;
    constraints_upperbound[y_start]    = y;
    constraints_upperbound[psi_start]  = psi;
    constraints_upperbound[v_start]    = v;
    constraints_upperbound[cte_start]  = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    if(solution.status == CppAD::ipopt::solve_result<Dvector>::success)
    {
        // Cost
        auto cost              = solution.obj_value;
        double last_xtrk_error = solution.x[epsi_start  - 1];
        double last_hdg_error  = Rad2Deg(solution.x[delta_start - 1]);

        m_delta = solution.x[delta_start];
        m_a     = solution.x[a_start];

        std::cout << "MPC Solution"
                  << ", Maximum speed: "       << MAX_SPEED
                  << ", Steps: "               << N
                  << ", Time step: "           << dt
                  << ", Cost: "                << cost
                  << ", XtrkError: "           << last_xtrk_error
                  << ", HdgError: "            << last_hdg_error
                  << ", Steer angle: "         << Rad2Deg(m_delta)
                  << ", Throttle: "            << m_a
                  << std::endl;

        m_solution_x.clear();
        m_solution_y.clear();

        for(unsigned int i = 0; i < N; ++i)
        {
            m_solution_x.push_back(solution.x[x_start + i]);
            m_solution_y.push_back(solution.x[y_start + i]);
        }
    }
    else
    {
        std::cout << "ERROR! MPC Solution UNSUCCESSFUL" << std::endl;
    }

    //Return the first actuator values.
    return {m_delta, m_a};
}

const vector<double>& MPC::GetSolutionX() const
{
    return m_solution_x;
}

const vector<double>& MPC::GetSolutionY() const
{
    return m_solution_y;
}

Eigen::VectorXd MPC::ProjectState(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, double dt) const
{
    Eigen::VectorXd projected(state.size());

    if(dt > 0.0)
    {
        double x    = state[0];
        double y    = state[1];
        double psi  = state[2];
        double v    = state[3];
        double cte  = state[4];
        double epsi = state[5];

        double delta_psi = v * m_delta * dt / Lf;

        double x2    = x    + v * cos(psi) * dt;
        double y2    = y    + v * sin(psi) * dt;
        double psi2  = psi  + delta_psi;
        double v2    = v    + m_a * dt;
        double cte2  = cte  + v * sin(epsi) * dt;
        double epsi2 = epsi + delta_psi;

        projected << x2, y2, psi2, v2, cte2, epsi2;

        std::cout << "ProjectState with delay " << dt << std::endl;
    }
    else
    {
        projected = state;

        std::cout << "ERROR! ProjectState with wrong delay " << dt << std::endl;
    }

    return projected;
}

