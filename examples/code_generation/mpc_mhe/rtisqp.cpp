/* Description: Source file for code generation of RTISQP solver in SAARTI
Run instructions: https://github.com/larsvens/acado_fork
*/
#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO


int main(int argc, char * const argv[ ])
{

    // Variables
    DifferentialState	s, d, deltapsi, psidot, vx, vy;
    OnlineData kappac, Cr, s_lb, s_ub, d_lb, d_ub;

    IntermediateState	Fyr;
    Control				Fyf, Fxf, Fxr;

    // slack variable
    DifferentialState   dummy;  // dummy state for slack variable
    Control             sv;


    const double		g = 9.81;

    // gotthard
//    const double		m = 190.0;
//    const double		Iz = 110;
//    const double		lf = 1.22;
//    const double		lr = 1.22;
//    // gotthard magic formula
//    const double B = 12.56;
//    const double C = 1.38; // sign?
//    const double D = 1.60;
//    const double Fztot = m*g;
//    const double w_rear = 0.5; // percentage weigt on rear axle
//    const double Cr = B*C*D*Fztot*w_rear; // Rajamani

    // rhino
    const double m = 8350.0;
    const double Iz = 8158.0;
    const double lf = 1.205;
    const double lr = 2.188;
    // rhino magic formula
    const double B = 10.0;
    const double C = 1.9;
    const double D = 1.00;
    const double Fztot = m*g;
    const double w_rear = 0.5; // percentage weigt on rear axle
    //const double Cr = B*C*D*Fztot*w_rear; // Rajamani
    //std::cout << "Cr = " << Cr << std::endl;

    //Fyr = 2*Cr*atan(lr*psidot-vy)/vx; //  atan ok?
    Fyr = 2*Cr*(lr*psidot-vy)/vx;

    // Differential algebraic equation
    DifferentialEquation f;
    f << dot(s) == (vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac);
    f << dot(d) == vx*sin(deltapsi)+vy*cos(deltapsi);
    f << dot(deltapsi) == psidot-kappac*(vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac);
    f << dot(psidot) == (1/Iz)*(lf*Fyf - lr*Fyr);
    f << dot(vx) == (1/m)*(Fxf+Fxr); // todo include bank, grade as onlinedata (and aero dyn)
    f << dot(vy) == (1/m)*(Fyf+Fyr)-vx*psidot;

    f << dot(dummy) == sv; // slack

    // Weighting matrices and reference functions
    Function rf;
    Function rfN;
    rf << s << d << deltapsi << psidot << vx << vy << dummy << Fyf << Fxf << Fxr << sv;
    rfN << s << d << deltapsi << psidot << vx << vy << dummy;

    // this notation for setting weights at runtime (DMatrix for setting here)
    BMatrix W = eye<bool>( rf.getDim() );
    BMatrix WN = eye<bool>( rfN.getDim() );

    // Optimal Control Problem
    const int N  = 40; // 30
    const int Ni = 5; // 5
    const double Ts = 0.1; // 0.1

    OCP ocp(0, N * Ts, N);
    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);


    // input constraints
    // use "inequalities_from_vertices.m to compute inequality coefficients"
    std::vector<double> a = {-0.00073205, -0.00026795, 0.00026795, 0.00073205, 0.001, 0.001, 0.00073205, 0.00026795, -0.00026795, -0.00073205, -0.001, -0.001};
    std::vector<double> b = {-0.00073205, -0.001, -0.001, -0.00073205, -0.00026795, 0.00026795, 0.00073205, 0.001, 0.001, 0.00073205, 0.00026795, -0.00026795};
    uint N_vertices = uint(a.size()); // (even) nr of vertices in input constraint polytope

    // FRONT WHEEL
    // loop over N_vertices/2 to set affine constraints on Fyf, Fxf representing friction circle
    // lb = -1 and ub = 1 represents a friction circle of radius 1kN
    // lb and ub are scaled up and down at runtime (by the n*N first entries in ubAValues)
    for (uint i = 0; i < N_vertices/2; ++i) {
        ocp.subjectTo(-1 <= Fxf*a.at(i) + Fyf*b.at(i) <= 1);
    }
    // front wheel drive constraint is set online


    // REAR WHEEL
    ocp.subjectTo(-1000 <= Fxr <= 1000); // changed online by ub values


    // alternative static box constraints on inputs
//    ocp.subjectTo(-1000 <= Fyf <= 1000);
//    ocp.subjectTo(-1000 <= Fxf <= 1000);
//    ocp.subjectTo(-1000 <= Fxr <= 1000);


    // state constraints (todo introduce slack)
    ocp.setNOD(6);    // must set NOD manually
//    ocp.subjectTo(s - s_lb + sv >= 0);
//    ocp.subjectTo(s_ub - s + sv >= 0);
//    ocp.subjectTo(d - d_lb + sv >= 0);
//    ocp.subjectTo(d_ub - d + sv >= 0);
    ocp.subjectTo(s - s_lb >= 0);
    ocp.subjectTo(s_ub - s >= 0);
    ocp.subjectTo(d - d_lb >= 0);
    ocp.subjectTo(d_ub - d >= 0);
    // Export:
    OCPexport mpc( ocp );
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4); // INT_IRK_GL2 fastest among integrators that work for this problem
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    //mpc.set(MAX_NUM_QP_ITERATIONS, 3);
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
