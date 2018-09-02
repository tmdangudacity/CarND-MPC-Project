#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
    public:

        MPC();
        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuations.
        vector<double> Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, double delay);

        const vector<double>& GetSolutionX() const;
        const vector<double>& GetSolutionY() const;

    private:

        double m_delta;
        double m_a;

        vector<double> m_solution_x;
        vector<double> m_solution_y;

        Eigen::VectorXd ProjectState(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, double dt) const;
};

#endif /* MPC_H */
