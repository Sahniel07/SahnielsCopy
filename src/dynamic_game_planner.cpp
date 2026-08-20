#include "dynamic_game_planner.h"
//#include "recorder.h"
// #include "update_trajetcory_interface.h"
#include <iostream>
#include <iomanip>
#include <string>

#define error 1e-3f

DynamicGamePlanner::DynamicGamePlanner() 
{
}

DynamicGamePlanner::~DynamicGamePlanner() {
    // Destructor implementation
    std::cout << "DynamicGamePlanner destroyed." << std::endl;
}

void DynamicGamePlanner::run(TrafficParticipants& traffic_state) {
    
    traffic = traffic_state;
    setup();
    //Copy data for ISPC
    copyDataForISPC(traffic);
    // definition of the control variable vector U and of the state vector X:
    double U_[nU_];
    double X_[nX_];
    SoA_X_Double X;
    SoA_U_Double U;
    double constraints[nC];
    double gradient[nU_];

    initial_guess(X, U);
    trust_region_solver(U);
    launch_integrate(&X, &U);
    convertBackData_U(U_, U);
    convertBackData_X(X_, X);
    print_trajectories(X_, U_);
    compute_constraints(constraints, X, U);
    // #ifdef USE_RECORDER
    //     for (int i = 0; i < nC; i++) {
    //         Recorder::getInstance()->saveData<double>("i", i);
    //         Recorder::getInstance()->saveData<double>("constraints", constraints[i]);
    //     }
	// #endif
    constraints_diagnostic(constraints, false);
    traffic = set_prediction(X_, U_);
}

void DynamicGamePlanner::setup() {
    
    // Setup number of traffic participants:
    M = traffic.size();
    
    // Setup number of inequality constraints for one vehicle:
    // 2 * nU * (N + 1) inequality constraints for inputs 
    // (N + 1) * (M - 1) collision avoidance constraints
    // (N + 1) constraints to remain in the lane
    nC_i = 2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1) + (param.N + 1);

    // Setup number of inequality constraints for all the traffic participants
    nC = nC_i * M;

    // Setup number of elements in the state vector X:
    // number of state variables * number of timesteps * number of traffic participants
    nX_ = param.nX * (param.N + 1) * M;

    // Setup number of elements in the control vector U:
    // number of control variables * number of timesteps * number of traffic participants
    nU_ = param.nU * (param.N + 1) * M;

    // Setup length of the gradient vector G:
    nG = nU_;

    // resize and initialize limits for control input
    ul.resize(param.nU * (param.N + 1), 1);
    uu.resize(param.nU * (param.N + 1), 1);
    for (int j = 0; j < param.N + 1; j++){
        ul(param.nU * j + d, 0) = param.d_low;
        uu(param.nU * j + d, 0) = param.d_up;
        ul(param.nU * j + F, 0) = param.F_low;
        uu(param.nU * j + F, 0) = param.F_up;
    }

    // resize time vector
    time.resize(param.N + 1, 1);

    // resize and initialize lagrangian multiplier vector
    lagrangian_multipliers.resize(nC, 1);
    lagrangian_multipliers = Eigen::MatrixXd::Zero(nC, 1);

}

/** Sets the intial guess of the game */
void DynamicGamePlanner::initial_guess(SoA_X_Double& X_, SoA_U_Double& U_)
{
    for(int i = 0; i < NUM_State; i++)
    {
        for(int j = 0; j < M; j++)
        {
            int index = i * MAX_OBSTACLES + j;
            U_.d[index] = 0.0;
            U_.F[index] = 0.0;
        }
    }

    launch_integrate(&X_, &U_);
    // #ifdef USE_RECORDER
    //     for (int i = 0; i < M; i++) {
    //         for (int j = 0; j < param.N + 1; j++) {
    //         Recorder::getInstance()->saveData<double>("i", i);
    //         Recorder::getInstance()->saveData<double>("j", j);    
    //         Recorder::getInstance()->saveData<double>("X_x", X_[param.nX * (param.N + 1) * i + param.nX * j + x]);
    //         Recorder::getInstance()->saveData<double>("X_y", X_[param.nX * (param.N + 1) * i + param.nX * j + y]);
    //         Recorder::getInstance()->saveData<double>("X_v", X_[param.nX * (param.N + 1) * i + param.nX * j + v]);
    //         Recorder::getInstance()->saveData<double>("X_psi", X_[param.nX * (param.N + 1) * i + param.nX * j + psi]);
    //         Recorder::getInstance()->saveData<double>("X_s", X_[param.nX * (param.N + 1) * i + param.nX * j + s]);
    //         Recorder::getInstance()->saveData<double>("X_l", X_[param.nX * (param.N + 1) * i + param.nX * j + l]);
    //         Recorder::getInstance()->saveData<double>("U_F", U_[param.nU * (param.N + 1) * i + param.nU * j + F]);
    //         Recorder::getInstance()->saveData<double>("U_d", U_[param.nU * (param.N + 1) * i + param.nU * j + d]);
    //         }
    //     }
	// #endif
}
/** integrates the input U to get the state X */
void DynamicGamePlanner::integrate(SoA_X* X_, const SoA_U* U_)
{
    int index;
    int ind;
    typeInC v_ref;
    typeInC t;
    typeInC s_t0[param.nX];
    typeInC u_t0[param.nU];
    typeInC ds_t0[param.nX];

    for (int i = 0; i < M; i++){
        ind = 0;
        t = 0.0;

        // Initial state:
        s_t0[x] = traffic[i].x / param.L_Max;
        s_t0[y] = traffic[i].y / param.L_Max;
        s_t0[v] = traffic[i].v / param.V_Max;
        s_t0[psi] = traffic[i].psi / param.ang_norm;
        s_t0[s] = 0.0;
        s_t0[l] = 0.0;

        for (int j = 0; j < param.N + 1; j++){
            index = MAX_OBSTACLES * j +  i;
            v_ref = traffic[i].v_target / param.V_Max;

            u_t0[d] = U_->d[index] / param.ang_norm;
            u_t0[F] = U_->F[index] * param.L_Max / (param.V_Max * param.V_Max * param.M_norm);

            // Derivatives: 
            typeInC angle_1(param.cg_ratio * u_t0[d]);
            typeInC angle = s_t0[psi] + angle_1;
            ds_t0[x] = s_t0[v] * cos(angle * param.ang_norm);
            ds_t0[y] = s_t0[v] * sin(angle * param.ang_norm);
            ds_t0[v] = (-1.0/param.tau) * (param.tau_norm) * s_t0[v] + (param.k * param.k_norm) * u_t0[F];
            ds_t0[psi] = s_t0[v] * tan(u_t0[d] * param.ang_norm) * cos(angle_1 * param.ang_norm)/ (param.length * param.length_norm * param.ang_norm);
            ds_t0[l] = param.weight_target_speed * param.weight_target_speed_norm * (s_t0[v] - v_ref) * (s_t0[v] - v_ref);
            ds_t0[s] = s_t0[v];

            // Integration to compute the new state: 
            s_t0[x] += param.dt * param.dt_norm * ds_t0[x];
            s_t0[y] += param.dt * param.dt_norm * ds_t0[y];
            s_t0[v] += param.dt * param.dt_norm * ds_t0[v];
            s_t0[psi] += param.dt * param.dt_norm * ds_t0[psi];
            s_t0[s] += param.dt * param.dt_norm * ds_t0[s];
            s_t0[l] += param.dt * param.dt_norm * ds_t0[l];

            if (s_t0[v] < 0.0){s_t0[v] = 0.0;}

            // Save the state in the trajectory
            X_->x[index] = s_t0[x];
            X_->y[index] = s_t0[y];
            X_->v[index] = s_t0[v];
            X_->psi[index] = s_t0[psi];
            X_->s[index] = s_t0[s];
            X_->l[index] = s_t0[l];
            t+= param.dt * param.dt_norm;    
        }
    }
}

/** SR1 Hessian matrix update*/
void DynamicGamePlanner::hessian_SR1_update(Eigen::MatrixXd & H_, const Eigen::MatrixXd & s_, const Eigen::MatrixXd & y_, double r_)
{
    if (abs((s_.transpose() * (y_ - H_ * s_))(0,0)) > 
                r_ * (s_.transpose() * s_)(0,0) * ((y_ - H_* s_).transpose() * (y_ - H_ * s_))(0,0))
    {
        H_ = H_ + ((y_ - H_ * s_) * (y_ - H_ * s_).transpose()) / ((y_ - H_ * s_).transpose() * s_)(0,0);
    }
}

/** function to increase rho = rho * gamma at each iteration */
void DynamicGamePlanner::increasing_schedule()
{
    rho = gamma * rho;
}

/** function to save the lagrangian multipliers in the general variable */
void DynamicGamePlanner::save_lagrangian_multipliers(double* lagrangian_multipliers_)
{
    for (int i = 0; i < nC; i++){
        lagrangian_multipliers(i,0) = lagrangian_multipliers_[i];
    }
}

/* computation of lambda (without update)*/
void DynamicGamePlanner::compute_lagrangian_multipliers(double* lagrangian_multipliers_, const double* constraints_)
{
    double l;
    for (int i = 0; i < nC; i++){
        l = lagrangian_multipliers(i,0) + rho * constraints_[i];
        lagrangian_multipliers_[i] = std::max(l, 0.0);
    }
}

/** computation of the inequality constraints (target: constraints < 0) */
void DynamicGamePlanner::compute_constraints(double* constraints, const SoA_X_Double& X_, const SoA_U_Double& U_)
{
    double constraints_i[nC_i];
    for (int i = 0; i < M; i++){
        compute_constraints_vehicle_i(constraints_i, X_, U_, i);
        for (int j = 0; j < nC_i; j++){
            constraints[nC_i * i + j] = constraints_i[j];
            #ifdef USE_RECORDER
            const std::string constraint_key = "con_vehicle=" + std::to_string(i) + "_typeofCons=" + std::to_string(j);
            Recorder::getInstance()->saveData<double>(constraint_key, constraints_i[j]);
            Recorder::getInstance()->saveData<double>("i", i);
            Recorder::getInstance()->saveData<double>("j", j);
            #endif
        }
    }
    
}

/** computation of the inequality constraints C for vehicle i (target: C < 0) */
void DynamicGamePlanner::compute_constraints_vehicle_i(double* constraints_i, const SoA_X_Double& X_, const SoA_U_Double& U_, int i)
{
    int ind = 0;
    int indCu;
    int indCl;
    int indCto;
    int indClk;
    int indCuk;
    int indf;
    int n1;
    int n2;
    double dist2t[param.N + 1];
    double rad2[param.N + 1];
    double latdist2t[param.N + 1];
    double r_lane_ = param.r_lane;

    // constraints for the inputs 
    indCu = param.nU * (param.N + 1);
    indCl = indCu + param.nU * (param.N + 1);
    for (int k = 0; k < param.N + 1; k++){
        constraints_i[param.nU * k + d] = 1e3 * (U_.d[k * MAX_OBSTACLES + i] - uu(param.nU * k + d,0));
        constraints_i[param.nU * k + F] = 1e3 * (U_.F[k * MAX_OBSTACLES + i] - uu(param.nU * k + F,0));
        constraints_i[indCu + param.nU * k + d] = 1e3 * (ul(param.nU * k + d,0) - U_.d[k * MAX_OBSTACLES + i]);
        constraints_i[indCu + param.nU * k + F] = 1e3 * (ul(param.nU * k + F,0) - U_.F[k * MAX_OBSTACLES + i]);
    }

    // collision avoidance constraints  
    for (int k = 0; k < M; k++){
        if (k != i){
            indCto = indCl + (param.N + 1) * ind;
            compute_squared_distances_vector(dist2t, X_, i, k);
            for (int j = 0; j < param.N + 1; j++){
                constraints_i[indCto + j] = (param.r_safe * param.r_safe - dist2t[j]);
            }
            ind++;
        }
    }
    indCto = indCl + (param.N + 1) * (M - 1);

    // constraints to remain in the lane
    compute_squared_lateral_distance_vector(latdist2t, X_, i);
    for (int k = 0; k < param.N + 1; k++){
        constraints_i[indCto + k] = (latdist2t[k] - r_lane_ * r_lane_);
    }
    indf = indCto + (param.N + 1);
}

/** computes a vector of the squared distance between the trajectory of vehicle i and j*/
void DynamicGamePlanner::compute_squared_distances_vector(double* squared_distances, const SoA_X_Double& X_, int ego, int j)
{
    double x_ego;
    double y_ego;
    double x_j;
    double y_j;
    double distance;
    for (int k = 0; k < param.N + 1; k++){
        x_ego = X_.x[k * MAX_OBSTACLES + ego];
        y_ego = X_.y[k * MAX_OBSTACLES + ego];
        x_j = X_.x[k * MAX_OBSTACLES + j];
        y_j = X_.y[k * MAX_OBSTACLES + j];
        distance = (x_ego - x_j) * (x_ego - x_j) + (y_ego - y_j) * (y_ego - y_j);
        squared_distances[k] = distance;
    }
}

/** computes a vector of the squared lateral distance between the i-th trajectory and the allowed center lines at each time step*/
void DynamicGamePlanner::compute_squared_lateral_distance_vector(double* squared_distances_, const SoA_X_Double& X_, int i)
{
    double s_;
    double x_;
    double y_;
    double x_c;
    double y_c;
    double x_r;
    double y_r;
    double x_l;
    double y_l;
    double psi_c;
    double psi_l;
    double psi_r;
    double dist_c;
    double dist_l = 1e3;
    double dist_r = 1e3;
    double dist_long_c;
    double dist_long_l;
    double dist_long_r;
    double dist2_c[param.N + 1];
    double dist2_l[param.N + 1];
    double dist2_r[param.N + 1];
    double dist2_rl_min;
    for (int j = 0; j < param.N + 1; j++){
        s_ = X_.s[j * MAX_OBSTACLES + i];
        x_ = X_.x[j * MAX_OBSTACLES + i];
        y_ = X_.y[j * MAX_OBSTACLES + i];
        dist2_c[j] = 1e3;
        dist2_l[j] = 1e3;
        dist2_r[j] = 1e3;
        if (s_ < traffic[i].centerlane.s_max){
            x_c = traffic[i].centerlane.spline_x(s_);
            y_c = traffic[i].centerlane.spline_y(s_);
            psi_c = traffic[i].centerlane.compute_heading(s_);
            dist_c = ((x_ - x_c) * (x_ - x_c) + (y_ - y_c) * (y_ - y_c));
            dist_long_c = ((x_ - x_c) * std::cos(psi_c) + (y_ - y_c) * std::sin(psi_c)) * ((x_ - x_c) * std::cos(psi_c) + (y_ - y_c) * std::sin(psi_c));
            dist2_c[j] = dist_c - dist_long_c;
        }
        if (traffic[i].leftlane.present == true && s_ < traffic[i].leftlane.s_max && traffic[i].leftlane.s_max > 10.0){
            x_l = traffic[i].leftlane.spline_x(s_);
            y_l = traffic[i].leftlane.spline_y(s_);
            psi_l = traffic[i].leftlane.compute_heading(s_);
            dist_l = ((x_ - x_l) * (x_ - x_l) + (y_ - y_l) * (y_ - y_l));
            dist_long_l = ((x_ - x_l) * std::cos(psi_l) + (y_ - y_l) * std::sin(psi_l)) * ((x_ - x_l) * std::cos(psi_l) + (y_ - y_l) * std::sin(psi_l));
            dist2_l[j] = dist_l - dist_long_l;
        }
        if (traffic[i].rightlane.present == true && s_ < traffic[i].rightlane.s_max && traffic[i].rightlane.s_max > 10.0){
            x_r = traffic[i].rightlane.spline_x(s_);
            y_r = traffic[i].rightlane.spline_y(s_);
            psi_r = traffic[i].rightlane.compute_heading(s_);
            dist_r = ((x_ - x_r) * (x_ - x_r) + (y_ - y_r) * (y_ - y_r));
            dist_long_r = ((x_ - x_r) * std::cos(psi_r) + (y_ - y_r) * std::sin(psi_r)) * ((x_ - x_r) * std::cos(psi_r) + (y_ - y_r) * std::sin(psi_r));
            dist2_r[j] = dist_r - dist_long_r;
        }
        dist2_rl_min = std::min(dist2_l[j], dist2_r[j]);
        squared_distances_[j] = std::min(dist2_rl_min, dist2_c[j]);
    }
}

/** compute the cost for vehicle i */
double DynamicGamePlanner::compute_cost_vehicle_i(const SoA_X_Double& X_, int i)
{
    double final_lagrangian = X_.l[param.N * MAX_OBSTACLES + i];
    double cost = 0.5 * final_lagrangian * qf * final_lagrangian;
    return cost;
}

/** computes of the augmented lagrangian vector  L = <L_1, ..., L_M> L_i = cost_i + lagrangian_multipliers * constraints */
void DynamicGamePlanner::compute_lagrangian(double* lagrangian, const SoA_X_Double& X_, const SoA_U_Double& U_)
{
    double lagrangian_i;
    double cost_i;
    double constraints_i[nC_i];
    double lagrangian_multipliers_i[nC_i];
    for (int i = 0; i < M; i++){
        cost_i = compute_cost_vehicle_i( X_, i);
        compute_constraints_vehicle_i(constraints_i, X_, U_, i);
        lagrangian_i = compute_lagrangian_vehicle_i( cost_i, constraints_i, i);
        lagrangian[i] = lagrangian_i;
    }
}

/** computation of the augmented lagrangian for vehicle i: lagrangian_i = cost_i + lagrangian_multipliers_i * constraints_i */
double DynamicGamePlanner::compute_lagrangian_vehicle_i(double cost_i, const double* constraints_i, int i)
{
    double lagrangian_i = cost_i;
    double constraints;
    for (int k = 0; k < nC_i; k++){
        constraints = std::max(0.0, constraints_i[k]);
        lagrangian_i += 0.5 * rho * constraints * constraints + lagrangian_multipliers(i * nC_i + k,0) * constraints_i[k];
    }
    return lagrangian_i;
}

/** computation of the gradient of lagrangian_i with respect to U_i for each i with parallelization on cpu*/
void DynamicGamePlanner::compute_gradient(double* gradient, const SoA_U_Double& U_)
{
    // const int num_threads = std::thread::hardware_concurrency();
    const int num_threads = 1;
    std::mutex mutex;
    std::vector<std::thread> threads(num_threads);

    // Definition of the work for each thread:
    auto computeGradient = [&](int start, int end) {
        SoA_U_Double dU;
        SoA_X_Double dX;
        SoA_X_Double X_;
        double lagrangian[M];
        double lagrangian_j;
        double cost_j;
        double constraints_j[nC_i];
        double lagrangian_multipliers_j[nC_i];
        int index;
        
        for(int i = 0; i < NUM_State; i++){
            for(int j=0; j<M;j++){
                int index = i * MAX_OBSTACLES + j;
                dU.d[index] = U_.d[index];
                dU.F[index] = U_.F[index];
            }
        }
        launch_integrate(&X_, &U_);
        SoA_X_Double X_before = X_;
        compute_lagrangian(lagrangian, X_, U_);
        for (int i = start; i < end; i++) {
            for(int j=0; j<M;j++){
                int index = i * MAX_OBSTACLES + j;
                dU.d[index] = U_.d[index] + param.eps;
                // #ifdef USE_RECORDER
                //     Recorder::getInstance()->saveData<float>("U_.d[index]", U_.d[index]);
                //     Recorder::getInstance()->saveData<float>("dU.d[index]", dU.d[index]);
                //     Recorder::getInstance()->saveData<float>("eps", param.eps);
                // #endif
                launch_integrate(&dX, &dU);
                compute_constraints_vehicle_i(constraints_j, dX, dU, j);
                cost_j = compute_cost_vehicle_i( dX, j);
                lagrangian_j = compute_lagrangian_vehicle_i( cost_j, constraints_j, j);
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    gradient[nu * j + i * param.nU + d] = (lagrangian_j - lagrangian[j]) / param.eps;
                }
                dU.d[index] = U_.d[index];

                dU.F[index] = U_.F[index] + param.eps;
                launch_integrate(&dX, &dU);
                // #ifdef USE_RECORDER
                // for (int i = 0; i < M; i++) {
                //     for (int j = 0; j < param.N + 1; j++) {
                //     Recorder::getInstance()->saveData<double>("i", i);
                //     Recorder::getInstance()->saveData<double>("j", j);    
                //     Recorder::getInstance()->saveData<double>("X_x_before", dX.x[(param.N + 1) * i + j]-X_before.x[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("X_y_before", dX.y[(param.N + 1) * i + j]-X_before.y[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("X_v_before", dX.v[(param.N + 1) * i + j]-X_before.v[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("X_psi_before", dX.psi[(param.N + 1) * i + j]-X_before.psi[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("X_s_before", dX.s[(param.N + 1) * i + j]-X_before.s[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("X_l_before", dX.l[(param.N + 1) * i + j]-X_before.l[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("U_F_before", dU.d[(param.N + 1) * i + j]-U_.d[(param.N + 1) * i + j]);
                //     Recorder::getInstance()->saveData<double>("U_d_before", dU.F[(param.N + 1) * i + j]-U_.F[(param.N + 1) * i + j]);
                //     }
                // }
                // #endif
                
                compute_constraints_vehicle_i(constraints_j, dX, dU, j);
                cost_j = compute_cost_vehicle_i( dX, j);
                lagrangian_j = compute_lagrangian_vehicle_i( cost_j, constraints_j, j);
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    gradient[nu * j + i * param.nU + F] = (lagrangian_j - lagrangian[j]) / param.eps;
                }
                dU.F[index] = U_.F[index];
            }
        }
    };

    // Parallelize:
    int work_per_thread = NUM_State / num_threads;
    int start_index = 0;
    int end_index = 0;
    for (int i = 0; i < num_threads; ++i) {
        start_index = i * work_per_thread;
        end_index = (i == num_threads - 1) ? NUM_State : start_index + work_per_thread;
        threads[i] = std::thread(computeGradient, start_index, end_index);
    }
    for (auto& thread : threads) {
        thread.join();
    }
}

/** it solves the quadratic problem (GT * s + 0.5 * sT * H * s) with solution included in the trust region ||s|| < Delta */
void DynamicGamePlanner::quadratic_problem_solver(Eigen::MatrixXd & s_, const Eigen::MatrixXd & G_, const Eigen::MatrixXd & H_, double Delta)
{
    Eigen::MatrixXd ps(nG,1);
    double tau;
    double tau_c;
    double normG;
    double GTHG;
    GTHG = (G_.transpose() * H_ * G_)(0,0);
    normG = sqrt((G_.transpose() * G_)(0,0));
    ps = - Delta * (G_ /(normG));
    if ( GTHG <= 0.0){
        tau = 1.0;
    }else{
        tau_c = (normG * normG * normG)/(Delta * GTHG);
        tau = std::min(tau_c, 1.0);
    }
    s_ = tau * ps;
}

/** prints if some constraints are violated */
void DynamicGamePlanner::constraints_diagnostic(const double* constraints, bool print = false)
{
    bool flag0 = false;
    bool flag1 = false;
    bool flag2 = false;
    bool flag3 = false;
    for (int i = 0; i < M; i++){
        flag0 = false;
        flag1 = false;
        flag2 = false;
        flag3 = false;
        for (int j = 0; j < nC_i; j++){
            if (constraints[nC_i * i +j] > 0){
                if (j < (2 * param.nU * (param.N + 1))) {
                    std::cerr<<"vehicle "<<i<<" violates input constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag0 = true;
                }
                if (j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)) && j > (2 * param.nU * (param.N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates collision avoidance constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag1 = true;
                }
                if (j > (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)) && j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1) + (param.N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates lane constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag2 = true;
                }
            }
        }
        if (print == true){
            std::cerr<<"vehicle "<<i<<"\n";
            std::cerr<<"input constraint: \n";
            for (int j = 0; j < 2 * param.nU * (param.N + 1); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\ncollision avoidance constraint: \n";
            for (int j = 2 * param.nU * (param.N + 1); j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\nlane constraint: \n";
            for (int j = (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1)); j < (2 * param.nU * (param.N + 1) + (param.N + 1) * (M - 1) + (param.N + 1)); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\n";
        }
    }
}

void DynamicGamePlanner::print_trajectories(const double* X, const double* U)
{
    // Define column width
    const int col_width = 12;  // Adjust this value as needed

    for (int i = 0; i < M; i++){
        std::cerr << "Vehicle: (" << traffic[i].x << ", " << traffic[i].y << ") \t" << traffic[i].v << "\n";

        // Print table header with aligned columns
        std::cerr << std::left  // Align text to the left
                  << std::setw(col_width) << "X"
                  << std::setw(col_width) << "Y"
                  << std::setw(col_width) << "V"
                  << std::setw(col_width) << "PSI"
                  << std::setw(col_width) << "S"
                  << std::setw(col_width) << "L"
                  << std::setw(col_width) << "F"
                  << std::setw(col_width) << "d"
                  << "\n";

        // Print separator line
        std::cerr << std::string(col_width * 8, '-') << "\n";

        // Print trajectory values
        for (int j = 0; j < param.N + 1; j++){
            std::cerr << std::fixed << std::left
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + x]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + y]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + v]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + psi]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + s]
                      << std::setw(col_width) << X[param.nX * (param.N + 1) * i + param.nX * j + l]
                      << std::setw(col_width) << U[param.nU * (param.N + 1) * i + param.nU * j + F]
                      << std::setw(col_width) << U[param.nU * (param.N + 1) * i + param.nU * j + d]
                      << "\n";
        }
        std::cerr << "\n";
    }
}

/** computes the acceleration on the spline s(t) at time t*/
double DynamicGamePlanner::compute_acceleration(const tk::spline & spline_v, double t)
{
    return spline_v.deriv(1, t);
}

/** sets the prediction to the traffic structure*/
TrafficParticipants DynamicGamePlanner::set_prediction(const double* X_, const double* U_)
{
    TrafficParticipants traffic_ = traffic;
    for (int i = 0; i < M; i++){
        Trajectory trajectory;
        Control control;
        double time = 0.0;

        for (int j = 0; j < param.N + 1; j++){
            TrajectoryPoint point;
            Input input;
            input.a = (-1/param.tau) * X_[ nx * i + param.nX * j + v] + (param.k) * U_[nu * i + param.nU * j + F];
            input.delta = U_[nu * i + param.nU * j + d];
            point.x = X_[ nx * i + param.nX * j + x];
            point.y = X_[ nx * i + param.nX * j + y];
            point.psi = X_[ nx * i + param.nX * j + psi];
            point.v = X_[ nx * i + param.nX * j + v];
            point.omega = point.v * tan(input.delta) * cos(param.cg_ratio * input.delta)/ param.length;
            point.beta = 0.5 * input.delta;
            point.l = X_[ nx * i + param.nX * j + l];
            point.s = X_[ nx * i + param.nX * j + s]; 
            point.t_start = time;
            point.t_end = time + param.dt;
            trajectory.push_back(point);
            control.push_back(input);
            time += param.dt;
        }
        traffic_[i].predicted_trajectory = trajectory;
        traffic_[i].predicted_control = control;
    }
    return traffic_;
}

/** computes the norm of the gradient */
double DynamicGamePlanner::gradient_norm(const double* gradient)
{
    double norm = 0.0;
    for (int j = 0; j < nG; j++){
        norm += gradient[j] * gradient[j];
    }
    return norm;
}

/** Trust-Region solver of the dynamic game*/
void DynamicGamePlanner::trust_region_solver(SoA_U_Double& U_)
{
    bool convergence = false;

    // Parameters:
    double eta = 1e-4;
    double r_ = 1e-8;
    double threshold_gradient_norm = M * 1e-3;
    int iter = 1;
    int iter_lim = 100;

    // Variables definition:
    double gradient[nG];
    SoA_U_Double dU; 
    SoA_U_Double dU_; 
    SoA_X_Double dX; 
    SoA_X_Double dX_;
    double d_gradient[nG];
    double d_lagrangian[M];
    double lagrangian[M];
    double constraints[nC];
    double lagrangian_multipliers[nC];

    double actual_reduction[M];
    double predicted_reduction[M];
    double delta[M];
    std::vector<Eigen::MatrixXd> H_(M);
    std::vector<Eigen::MatrixXd> g_(M);
    std::vector<Eigen::MatrixXd> p_(M);
    std::vector<Eigen::MatrixXd> s_(M);
    std::vector<Eigen::MatrixXd> y_(M);

    // Variables initialization:
    launch_integrate(&dX_, &dU_);

    for(int i = 0; i < NUM_State; i++){
        for(int j=0; j<M;j++){
            int index = i * MAX_OBSTACLES + j;
            dU.d[index] = U_.d[index];
            dU.F[index] = U_.F[index];
            dU_.d[index] = U_.d[index];
            dU_.F[index] = U_.F[index];
            dX_.x[index] = dX.x[index];
            dX_.y[index] = dX.y[index];
            dX_.v[index] = dX.v[index];
            dX_.psi[index] = dX.psi[index];
            dX_.s[index] = dX.s[index];
            dX_.l[index] = dX.l[index];
        }
    }
    for (int i = 0; i < M; i++){
        H_[i].resize(nu, nu);
        g_[i].resize(nu, 1);
        p_[i].resize(nu, 1);
        s_[i].resize(nu, 1);
        y_[i].resize(nu, 1);
        delta[i] = 1.0;
        H_[i] = Eigen::MatrixXd::Identity(nu, nu);
    }
    compute_gradient(gradient, dU_);

    // Check for convergence:
    if (gradient_norm(gradient) < threshold_gradient_norm){
        convergence = true;
    }

    // Iteration loop:
    while (convergence == false && iter < iter_lim ){

        // Compute the grandient and the lagrangian
        launch_integrate(&dX_, &dU_);
        compute_gradient(gradient, dU_);
        compute_lagrangian(lagrangian, dX_, dU_);

        // Solves the quadratic subproblem and compute the possible step dU:
        for (int i = 0; i < M; i++){
            for (int j = 0; j < param.N + 1; j++){
                g_[i](j * param.nU + d,0) = gradient[nu * i + j * param.nU + d];
                g_[i](j * param.nU + F,0) = gradient[nu * i + j * param.nU + F];
            }
            quadratic_problem_solver(s_[i], g_[i], H_[i], delta[i]);
            for (int j = 0; j < param.N + 1; j++){
                dU.d[j * MAX_OBSTACLES + i] = dU_.d[j * MAX_OBSTACLES + i] + s_[i](j * param.nU + d,0);
                dU.F[j * MAX_OBSTACLES + i] = dU_.F[j * MAX_OBSTACLES + i] + s_[i](j * param.nU + F,0);
            }
        }

        // Compute the new grandient and the new lagrangian with the possible step dU:
        launch_integrate(&dX, &dU);
        compute_gradient(d_gradient, dU);
        compute_lagrangian(d_lagrangian, dX, dU);

        // Check for each agent if to accept the step or not:
        for (int i = 0; i < M; i++){
            
            // Compute the actual reduction and of the predicted reduction:
            actual_reduction[i] = lagrangian[i] - d_lagrangian[i];
            predicted_reduction[i] = - (g_[i].transpose() *  s_[i] + 0.5 * s_[i].transpose() * H_[i] * s_[i])(0,0);

            // In case of very low or negative actual reduction, reject the step:
            if ( actual_reduction[i] / predicted_reduction[i] < eta){ 
                for (int j = 0; j < param.N + 1; j++){
                    dU.d[j * MAX_OBSTACLES + i] = dU_.d[j * MAX_OBSTACLES + i];
                    dU.F[j * MAX_OBSTACLES + i] = dU_.F[j * MAX_OBSTACLES + i];
                }
            }

            // In case of great reduction, and solution close to the trust region, increase the trust region:
            if ( actual_reduction[i] / predicted_reduction[i] > 0.75){ 
                if (std::sqrt((s_[i].transpose() * s_[i])(0,0)) > 0.8 * delta[i]){
                    delta[i] = 2.0 * delta[i];
                }
            }

            // In case of low actual reduction, decrease the step:
            if ( actual_reduction[i] / predicted_reduction[i] < 0.1){
                delta[i] = 0.5 * delta[i];
            }

            // Compute the difference of the gradients, then the Hessian matrix update:
            for (int j = 0; j < param.N + 1; j++){
                y_[i](j * param.nU + d,0) = d_gradient[nu * i + j * param.nU + d] - gradient[nu * i + j * param.nU + d];
                y_[i](j * param.nU + F,0) = d_gradient[nu * i + j * param.nU + F] - gradient[nu * i + j * param.nU + F];
            }
            hessian_SR1_update(H_[i], s_[i], y_[i], r_);

            // Save the solution for the next iteration:
            for (int j = 0; j < param.N + 1; j++){
                dU_.d[j * MAX_OBSTACLES + i] = dU.d[j * MAX_OBSTACLES + i];
                dU_.F[j * MAX_OBSTACLES + i] = dU.F[j * MAX_OBSTACLES + i];
            }
        }
         // Check for convergence:
        if (gradient_norm(gradient) < threshold_gradient_norm){
            convergence = true;
        }

        // Compute the new state:
        launch_integrate(&dX_, &dU_);

        // Compute the constraints with the new solution:
        compute_constraints(constraints, dX_, dU_);

        // Compute and save in the general variable the lagrangian multipliers with the new solution:
        compute_lagrangian_multipliers(lagrangian_multipliers, constraints);
        save_lagrangian_multipliers(lagrangian_multipliers);

        // Increase the weight of the constraints in the lagrangian multipliers:
        increasing_schedule();
        iter++;
    }

    std::cerr<<"number of iterations: "<<iter<<"\n";

    //Correct the final solution:
    correctionU(dU_);

    // Save the solution:
    for(int i = 0; i < NUM_State; i++){
        for(int j=0; j<M;j++){
            int index = i * MAX_OBSTACLES + j;
            U_.d[index] = dU_.d[index];
            U_.F[index] = dU_.F[index];
        }
    }
}

void DynamicGamePlanner::correctionU(SoA_U_Double& U_)
{
    
    for (int j = 0; j < param.N + 1; j++){
        for (int i = 0; i < M; i++){
            int index = j * MAX_OBSTACLES + i;
            if (j == param.N){
                U_.d[index] = U_.d[(j - 1) * MAX_OBSTACLES + i];
                U_.F[index] = U_.F[(j - 1) * MAX_OBSTACLES + i];
            }
            if (U_.d[index] > param.d_up){
                U_.d[index] = param.d_up;
            }
            if (U_.d[index] < param.d_low){
                U_.d[index] = param.d_low;
            }
        }
    }
}

void DynamicGamePlanner::copyDataForISPC(const TrafficParticipants& traffic)
{   
    //const double* data = traffic[0].centerlane.spline_x.get_x().data();
    // const size_t size = traffic[0].centerlane.spline_x.get_x().size();
    // for(int i=0;i<size;i++)
    // {
    //     std::cout << data[i] << "; ";
    // }
    for (int i = 0; i < M; i++) {
        state_ispc.x[i] = traffic[i].x;
        state_ispc.y[i] = traffic[i].y;
        state_ispc.v[i] = traffic[i].v;
        state_ispc.psi[i] = traffic[i].psi;
        state_ispc.v_target[i] = traffic[i].v_target;
    }
}

void DynamicGamePlanner::convertBackData_X(double* X_, SoA_X_Double& input_X_)
{
    for(int i = 0; i < NUM_State; i++)
    {
        for(int j=0; j<M;j++)
        {
            int index = i * MAX_OBSTACLES + j;
            int pos_X = Parameters::nX*NUM_State*j + Parameters::nX*i;
            X_[pos_X+x] = input_X_.x[index];
            X_[pos_X+y] = input_X_.y[index];
            X_[pos_X+v] = input_X_.v[index];
            X_[pos_X+psi] = input_X_.psi[index];
            X_[pos_X+s] = input_X_.s[index];
            X_[pos_X+l] = input_X_.l[index];
        }
    }
}

void DynamicGamePlanner::convertBackData_U(double* U_, SoA_U_Double& input_U_)
{
    for(int i = 0; i < NUM_State; i++)
    {
        for(int j=0; j<M;j++)
        {
            int index = i * MAX_OBSTACLES + j;
            int pos_U = Parameters::nU*NUM_State*j + Parameters::nU*i;
            U_[pos_U+d] = input_U_.d[index];
            U_[pos_U+F] = input_U_.F[index];
        }
    }
}

void DynamicGamePlanner::convertToISPC(const SoA_U_Double* U_Double, const SoA_X_Double* X_Double, SoA_U* U_, SoA_X* X_)
{
    for(int i=0; i<NUM_State*MAX_OBSTACLES; i++)
    {
        U_->d[i] = (typeInC)(U_Double->d[i]);
        U_->F[i] = (typeInC)(U_Double->F[i]);
        X_->x[i] = (typeInC)(X_Double->x[i]);
        X_->y[i] = (typeInC)(X_Double->y[i]);
        X_->psi[i] = (typeInC)(X_Double->psi[i]);
        X_->v[i] = (typeInC)(X_Double->v[i]);
        X_->s[i] = (typeInC)(X_Double->s[i]);
        X_->l[i] = (typeInC)(X_Double->l[i]);
    }
}

void DynamicGamePlanner::convertFromISPC(const SoA_X* X_, SoA_X_Double* X_Double)
{
    for(int i=0; i<NUM_State*MAX_OBSTACLES; i++)
    {
        X_Double->x[i] = (double)(X_->x[i]) * param.L_Max;
        X_Double->y[i] = (double)(X_->y[i]) * param.L_Max;
        X_Double->psi[i] = (double)(X_->psi[i]) * param.ang_norm;
        X_Double->v[i] = (double)(X_->v[i]) * param.V_Max;
        X_Double->s[i] = (double)(X_->s[i]) * param.L_Max;
        X_Double->l[i] = (double)(X_->l[i]) * param.C_Max;
    }
}

// Superseded by convertFromISPC: with the scale factors gated in parameters.h,
// ENABLE_QUANTIZATION=0 sets L_Max/V_Max/C_Max/ang_norm to 1.0, so multiplying
// by them is a bit-exact no-op and convertFromISPC covers both modes.
// void DynamicGamePlanner::convertFromISPC_WithoutQuantization(const SoA_X* X_, SoA_X_Double* X_Double)
// {
//     for(int i=0; i<NUM_State*MAX_OBSTACLES; i++)
//     {
//         X_Double->x[i] = (double)(X_->x[i]);
//         X_Double->y[i] = (double)(X_->y[i]);
//         X_Double->psi[i] = (double)(X_->psi[i]);
//         X_Double->v[i] = (double)(X_->v[i]);
//         X_Double->s[i] = (double)(X_->s[i]);
//         X_Double->l[i] = (double)(X_->l[i]);
//     }
// }

void DynamicGamePlanner::launch_integrate(SoA_X_Double* X_Double, const SoA_U_Double* U_Double)
{
    SoA_X X_;
    SoA_U U_;
    
    auto start = std::chrono::steady_clock::now();
    convertToISPC(U_Double, X_Double, &U_, &X_);
    // integrate_ispc(&X_, &U_, state_ispc, M);
    integrate(&X_, &U_); //this is the scalar version
        
    convertFromISPC(&X_, X_Double);
    auto end = std::chrono::steady_clock::now();

    sum_time_integration += end - start;
    number_calls++;
}