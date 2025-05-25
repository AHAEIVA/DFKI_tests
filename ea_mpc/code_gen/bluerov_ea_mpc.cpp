 
/**
*    \author Mohit Mehindratta / Hakim Amer
*    \date   1/9/2022
Acado ccode for mpc implementation for controlling the bluerov2 in 2-D. The model used can be found in the paper "Super-twisting integral sliding mode control for trajectory tracking of an
Unmanned Underwater Vehicle"
*/
 
 
#include <acado_code_generation.hpp>

#include <boost/algorithm/string.hpp>
 
USING_NAMESPACE_ACADO
 
 
int main()
{
 fff
    //State Variables:
    DifferentialState x;  // the body position w.r.t X_I
    DifferentialState y;  // the body position w.r.t Y_I
    DifferentialState z;  // the body position w.r.t Z_I
 
    DifferentialState u;  // the translation velocity along X_B
    DifferentialState v;  // the translation velocity along Y_B
    DifferentialState w;  // the translation velocity along Z_B
   
    DifferentialState psi;  // yaw angle
    DifferentialState r;   // yaw rate
 
   
    DifferentialState aux_state_px;
    DifferentialState aux_state_py;
    DifferentialState aux_state_pz;
 
 
    DifferentialState aux_state_ox;
    DifferentialState aux_state_oy;
    DifferentialState aux_state_oz;
 
    DifferentialState aux_slack;
 
    OnlineData pe_x;  // entagelemnent point_x
    OnlineData pe_y;  // entagelemnent point_y
    OnlineData pe_z;  // entagelemnent point_z
 
    OnlineData ob_x;  // obstacle centre_x
    OnlineData ob_y;  // obstacle centre_y
    OnlineData ob_z;  // obstacle centre_z
 
 
 
    //OnlineData Fx_dist;  // the external disturbance force along X_B
    //OnlineData Fy_dist;  // the external disturbance force along Y_B
    //OnlineData Fz_dist;  // the external disturbance force along Z_B
    // MPC control input
    Control X;  // Force along X_B
    Control Y;  // Force along Y_B
    Control Z;  // Force along Z_B
    Control M_z;  // Torque about Z_B (Yawing moment)
 
    Control slack;
 
     
 
    // BlueROV2 Model Parameters
   
    const double m = 11.4;    // BlueROV2 mass (kg)  
    const double g = 9.82;  // gravitational field strength (m/s^2)
 
    const double F_bouy = 1026 * 0.0115 * g; // Bouyancy force (N)
    const double eps = 0.00001;
    //const double F_bouy = 114.8; // Buoyancy force (N)
 
    const double X_ud = -2.6 ; // Added mass in x direction (kg)
    const double Y_vd = -18.5 ; // Added mass in y direction (kg)
    const double Z_wd = -13.3 ; // Added mass in z direction (kg)
    const double N_rd = -0.28 ; // Added mass for rotation about z direction (kg)
 
    const double I_xx = 0.21 ; // Moment of inertia (kg.m^2)
    const double I_yy = 0.245 ; // Moment of inertia (kg.m^2)
    const double I_zz = 0.245 ; // Moment of inertia (kg.m^2)
 
    const double X_u = -0.09 ; // Linear damping coefficient in x direction (N.s/m)
    const double Y_v  = -0.26 ; // Linear damping coefficient  in y direction (N.s/m)
    const double Z_w = -0.19; // Linear damping coefficient  in z direction (N.s/m)
    const double N_r = -4.64 ;  // Linear damping coefficient for rotation about z direction (N.s/rad)
 
    const double X_uc = -34.96 ; // quadratic damping coefficient in x direction (N.s^2/m^2)
    const double Y_vc = -103.25 ; // quadratic damping coefficient  in y direction (N.s^2/m^2)
    const double Z_wc = -74.23 ; // quadratic damping coefficient  in z direction (N.s^2/m^2)
    const double N_rc = - 0.43 ; // quadratic damping coefficient for rotation about z direction (N.s^2/rad^2)
 
    
    double d_safe = 2.0;  // Minimum distance from each side of the square
 
    // Model equations: 2-D model, assuming no roll or pitch
    DifferentialEquation f;
 
    f << dot(x) == cos(psi) * u - sin(psi) * v;
    f << dot(y) == sin(psi) * u +  cos(psi) * v;
    f << dot(z) ==  w;
 
    f << dot(u) == (X + (m * v - Y_vd * v) * r + (X_u + X_uc *sqrt( u * u + eps) ) * u)/(m - X_ud);
    f << dot(v) == (Y - (m * u - X_ud * u) * r + (Y_v + Y_vc *sqrt( v * v + eps) ) * v)/(m - Y_vd)  ;
    f << dot(w) == (Z + (Z_w + Z_wc * sqrt(w * w + eps)) * w + (m * g - F_bouy))/(m - Z_wd)  ;
 
    f << dot(psi) ==  r;
    f << dot(r) == (M_z - (m * v - Y_vd * v) * u - (X_ud * u - m * u) * v + (N_r + N_rc * sqrt(r * r + eps)) * r)/(I_zz - N_rd);
 
    f << dot(aux_state_px) == pe_x ;
    f << dot(aux_state_py) == pe_y;
    f << dot(aux_state_pz) == pe_z;
 
    f << dot(aux_state_ox) == ob_x;
    f << dot(aux_state_oy) == ob_y;
    f << dot(aux_state_oz) == ob_z;
 
    f << dot(aux_slack) == slack;
 
 
    // calculate entagelement cost

    //dot product with point e
    
    //calculate global velocity vector
    IntermediateState v_x =  cos(psi) * u - sin(psi) * v;
    IntermediateState v_y =  sin(psi) * u +  cos(psi) * v;

    //IntermediateState v_x =  dot(x);
    //IntermediateState v_y =  dot(y);
 
    IntermediateState r_px = x - ob_x;    // x component of position vector
    IntermediateState r_py = y - ob_y;    // y component of position vector


    IntermediateState dot_product = (pe_x * v_x + pe_y * v_y);
    IntermediateState mag_p = sqrt(pe_x * pe_x + pe_y * pe_y + eps);
    IntermediateState mag_v = sqrt(v_x * v_x + v_y * v_y + eps);
    IntermediateState mag_r = sqrt(r_px * r_px + r_py * r_py + eps);

    IntermediateState linear_cost = -(1- dot_product / (mag_p * mag_v));   //c1
    //c_ent = (pe_x - x) * (pe_x - x) + (pe_y - y) * (pe_y - y); ;
     
   ///// second cost
    //attract toward obstacle


    IntermediateState dist_obs = (x - ob_x) * (x - ob_x) + (y - ob_y) * (y - ob_y);


    IntermediateState dist_pe = (x - pe_x) * (x - pe_x) + (y - pe_y) * (y - pe_y);


    IntermediateState c2 = dist_obs * linear_cost;

    //Intermadiate c3 = dis_obs * linear_cost;



    //rotational cost
   // IntermediateState v_x =  cos(psi) * u - sin(psi) * v;
    //IntermediateState v_y =  sin(psi) * u +  cos(psi) * v;


    IntermediateState v_px = v_x;    // x component of velocity vector
    IntermediateState v_py = v_y;    // y component of velocity vector

    // Compute magnitudes of position and velocity vectors
   
    //IntermediateState mag_v = sqrt(v_px * v_px + v_py * v_py + eps);

    // Normalize position vector
    IntermediateState r_norm_x = r_px / mag_r;
    IntermediateState r_norm_y = r_py / mag_r;

    // Normalize velocity vector
    IntermediateState v_norm_x = v_px / mag_v;
    IntermediateState v_norm_y = v_py / mag_v;

    // Cross product (scalar) of the normalized vectors
    IntermediateState cross_prod = r_norm_x * v_norm_y - r_norm_y * v_norm_x;

    IntermediateState smooth_sign = (exp(2 * cross_prod +0.001 ) - 1) /
                                (exp(2 * cross_prod +0.001) + 1);

    //velocity cost
     mag_p = pe_x * pe_x + pe_y * pe_y;

    IntermediateState v_cost = 0.5 * ((u * u) + (v * v) - mag_p );  


    //IntermediateState mag_goal = (pe_z - x) * (pe_z - x) + (ob_z - y)*(ob_z - y);
    

    //IntermediateState rot_cost =  0.001 * (1 / dist_obs) * ( - cross_prod + v_cost);

    IntermediateState rot_cost =   - cross_prod ;




    //IntermediateState c_ent =  rot_cost + linear_cost + c2 ;

     //IntermediateState c_ent =  10.0* linear_cost + c2 +  1.0 * rot_cost - dist_obs ;
    
    //IntermediateState dist_cost = (x - ob_x) * (x - ob_x) + (y - ob_y) * (y - ob_y) - (d_safe +2.0) * (d_safe +2.0 );
    
    IntermediateState dist_cost = (x - ob_x) * (x - ob_x) + (y - ob_y) * (y - ob_y) - (ob_z * ob_z);

   // IntermediateState c_ent =  20.0 * linear_cost +   0.0 * pe_z * rot_cost +   0.0 * dist_cost +   0.0 *v_cost +
    //                   pe_z * x + pe_z * y ;

    

   //working
   //IntermediateState c_ent =  20.0 * linear_cost +  pe_z * ( x * x) + pe_z * (y * y) ;
   IntermediateState c_ent = 50.0 * linear_cost +  0.1 * pe_z  * linear_cost +  pe_z * ( x * x) + pe_z * (y * y) ;

     //c_ent = 10.0* linear_cost + 1.0 * rot_cost;
    
     //determine rotation direction
      // r cross pe
    //double ox = pe_z  - ob_x;
    // oy = ob_z  - ob_y

     //IntermediateState cross_prod = pe_x * ox - pe_y * oy;


     //c_ent = 2.0 * linear_cost + 0.01 * rot_cost + 20.0 * dist_pe;

    Function h, hN;
 
   
    h << x << y << z << u << v << w << psi << r << c_ent << X << Y<< Z << M_z << slack ;
    hN << x << y << z << u << v << w << psi << r;
 
 
    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());
 
    //
    // Optimal Control Problem
    //
    double N = 40;
    double Ts = 0.02;
    OCP ocp(0.0, N * Ts, N);
 
    ocp.subjectTo(f);
 
    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
 
 
   // Constraints on Inputs
    ocp.subjectTo(-80 <= X <= 80);
    ocp.subjectTo(-80  <= Y <= 80);
    ocp.subjectTo(-160 <= Z <= 160);
    ocp.subjectTo(-160 <= M_z <= 160);    //in Nm
    
    ocp.subjectTo((x - ob_x) * (x - ob_x) + (y - ob_y) * (y - ob_y) + eps - (d_safe - slack) * (d_safe - slack) >= 0);
    ocp.subjectTo( slack >= 0 );
 
 
 
// Constraints to keep (x, y) outside a square centered at (ob_x, ob_y)
//ocp.subjectTo((y - (ob_y + d_min)) >= 0);  // Above the top side
//ocp.subjectTo((y - (ob_y - d_min)) <= 0);  // Below the bottom side
//ocp.subjectTo((x - (ob_x + d_min)) >= 0);  // Right of the right side
//ocp.subjectTo((x - (ob_x - d_min)) <= 0);  // Left of the left side
 
    //ocp.subjectTo( slack >= 0 );
 
   // double eps = 0.0001;  // Small tolerance value
    // Export the code:
    OCPexport mpc(ocp);
 
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK4);     // RK4 for best comprimise between accuracy and computational effort
    mpc.set(NUM_INTEGRATOR_STEPS, 2 * N);
 
    mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(MAX_NUM_QP_ITERATIONS, 1000);
 
    //  mpc.set( SPARSE_QP_SOLUTION, SPARSE_SOLVER );
    //  mpc.set( QP_SOLVER, QP_QPDUNES );
 
    //mpc.set(HOTSTART_QP, YES);
 
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);  // Possible to Change Constraints Afterwards (only with qpOASES)
 
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);
 
    // Optionally set custom module name:
    mpc.set(CG_MODULE_NAME, "nmpc");
    mpc.set(CG_MODULE_PREFIX, "NMPC");
 
 
     

    std::string path_dir = "/home/hakim/tether_planning_ws/src/bluerov2_nmpc/solver";          

 
    mpc.printDimensionsQP();
 
 
    return EXIT_SUCCESS;
}
 