/* Description: Source file for code generation of RTISQP solver in SAARTI
Run instructions: https://github.com/larsvens/acado_fork
*/
#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

// todo read params from file

std::vector<double> linspace(double min, double max, int n)
{
    std::vector<double> result;
    // vector iterator
    int iterator = 0;

    for (int i = 0; i <= n-2; i++)
    {
        double temp = min + i*(max-min)/(floor((double)n) - 1);
        result.insert(result.begin() + iterator, temp);
        iterator += 1;
    }

    result.insert(result.begin() + iterator, max);
    return result;
}

int main(int argc, char * const argv[ ])
{

    // Variables
    DifferentialState	s, d, deltapsi, psidot, vx, vy;
    OnlineData kappac, s_lb, s_ub, d_lb, d_ub;

    IntermediateState	Fyr;
    Control				Fyf, Fx;

    // slack variable
    DifferentialState   dummy;  // dummy state for slack variable
    Control             sv;

    // ETH FS gottard
    const double		m = 190.0; // todo read from file
    const double		Iz = 110;
    const double		lf = 1.22;
    const double		lr = 1.22;
    const double        Crtilde = 100000; // todo get dynamically from fiala model (onlinedata)

    // Stanford Audi
//    const double		m = 1500; // todo read from file
//    const double		Iz = 2250;
//    const double		lf = 1.04;
//    const double		lr = 1.42;
//    const double        Crtilde = 180000; // todo get dynamically from fiala model (onlinedata)


    Fyr = 2*Crtilde*atan(lr*psidot-vy)/vx; //  atan ok?

    // Differential algebraic equation
    DifferentialEquation f;
    f << dot(s) == (vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac);
    f << dot(d) == vx*sin(deltapsi)+vy*cos(deltapsi);
    f << dot(deltapsi) == psidot-kappac*(vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac);
    f << dot(psidot) == (1/Iz)*(lf*Fyf - lr*Fyr);
    f << dot(vx) == (1/m)*Fx;
    f << dot(vy) == (1/m)*(Fyf+Fyr)-vx*psidot;
    f << dot(dummy) == sv; // slack

    // Weighting matrices and reference functions
    Function rf;
    Function rfN;
    rf << s << d << deltapsi << psidot << vx << vy << dummy << Fyf << Fx << sv;
    rfN << s << d << deltapsi << psidot << vx << vy << dummy;

    // this notation for setting weights at runtime (DMatrix for setting here)
    BMatrix W = eye<bool>( rf.getDim() );
    BMatrix WN = eye<bool>( rfN.getDim() );

    // Optimal Control Problem
    const int N  = 50; // 10
    const int Ni = 10; // 4
    const double Ts = 0.1;

    OCP ocp(0, N * Ts, N);
    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);


    // input constraints

    // use "inequalities_from_vertices.m to compute inequality coefficients"
    std::vector<double> a = {-4.1421e-05, 4.1421e-05, 0.0001, 0.0001, 4.1421e-05, -4.1421e-05, -0.0001, -0.0001, }; // 10000/5000
    std::vector<double> b = {-0.0002, -0.0002, -8.2843e-05, 8.2843e-05, 0.0002, 0.0002, 8.2843e-05, -8.2843e-05, };
    //std::vector<double> c = {1, 1, 1, 1, 1, 1, 1, 1, }; // always 1
    uint n = uint(a.size()); // (even) nr of vertices in input constraint polytope



    for (uint i = 0; i < n; ++i) {
        ocp.subjectTo(Fx*a.at(i) + Fyf*b.at(i) <= 1); // RHS is scaled at runtime (by the n*N first entries in ubAValues)
    }

    // tmp! (remove when polytope works)
    //ocp.subjectTo(-5000 <= Fyf <= 5000);
    //ocp.subjectTo(-10000 <= Fx <= 5000);


    // state constraints (todo introduce slack)
    ocp.setNOD(5);    // must set NOD manually
    ocp.subjectTo(s - s_lb + sv >= 0);
    ocp.subjectTo(s_ub - s + sv >= 0);
    ocp.subjectTo(d - d_lb + sv >= 0);
    ocp.subjectTo(d_ub - d + sv >= 0);

    // Export:
    OCPexport mpc( ocp );
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    // TMP! mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL2); // fastest among integrators that work for this problem
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
//	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
//	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
    mpc.set(HOTSTART_QP, YES);
//	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(USE_SINGLE_PRECISION, YES);
//	mpc.set(CG_USE_OPENMP, YES);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);

  if (mpc.exportCode( "rtisqp_export" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
