//
// Created by David on 08/02/2022.
//

#include "iLQR_funcs.h"

/**************************************************************************
 *
 *  iLQR Parameters
 *
 *
 */
float horizonLength = 10.0; // seconds
float dt = 0.04; // time between controls changing
int numControls = horizonLength / dt; // number of controls needed as per horizon length and time step
int mujoco_steps_per_dt = (dt / MUJOCO_TIMESTEP) + 1; // Number of mj_steps needed per control time step
int linearising_num_sim_steps = 2;  // How many mujoco steps to use for linearising
int num_controls_per_linearisation = 1;
bool alphaSearchEnabled = false;     // Whether alpha search is enabled to maximise optimisation at each forwards pass
float maxLamda = 10000;             // Maximum lambda before canceliing optimisation
float minLamda = 0.00001;            // Minimum lamda
float lamdaFactor = 10;             // Lamda multiplicative factor
float epsConverge = 0.001;
bool oneSidedFiniteDiff = true;
bool costFunctionFD = false;


//float controlCost[DOF] = {0.0001, 0.0001, 0.0001, 0.0001, 0.00005, 0.00005, 0.00005};
float controlCost[DOF] = {0, 0, 0, 0, 0, 0, 0};
float stateCosts[NUM_STATES] = {0, 0, 0, 0, 0, 0, 0,
                                0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01,
                                    100, 100};
//float termStateCosts[DOF] = {100, 100, 100, 100, 10, 10, 10};
float termStateCosts[DOF] = {1, 1, 1, 1, 1, 1, 1};
float accelCosts[DOF] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

/**************************************************************************
 *
 *  Other variables
 *
 *
 */

int torqueLims[DOF] = {87, 87, 87, 87, 12, 12, 12};
float angleLims[DOF] = {2.9, 1.76, 2.89, 1.5, 2.9, 1.89, 2.9};
float velocityLims[DOF] = {2.175, 2.175, 2.175, 2.175, 2.6, 2.6, 2.6};

int baseStateIndex = 1;
int initStateIndex = 0;

m_state X_desired;
m_dof_dof R;
m_state_state Q;
m_state_state Q_term;
m_dof_dof Z;

ofstream outputFile;

std::string filename = "iLQR.csv";


extern MujocoController *globalMujocoController;


// Make a linear model of the dynamics of the system at the current state
// Create an approximation of the type x(t.) = Ax(t) + Bu(t)
// This function calculates the A and B matrices using finite differencing
void lineariseDynamics(Ref<m_state> currentState, Ref<m_dof> currentControls, Ref<MatrixXf> A, Ref<MatrixXf> B){

    float epsState[NUM_STATES] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1, 0.1};
    float epsControls[DOF] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};

    auto linDynStart = high_resolution_clock::now();
    int microSecs_Loading = 0;
    auto startTimerLoading = high_resolution_clock::now();
    auto stopTimerLoading = high_resolution_clock::now();
    auto loadingDuration = duration_cast<microseconds>(stopTimerLoading - startTimerLoading);
    int numberMujocoStepsNeeded = linearising_num_sim_steps;

    // Make a copy of the current state
    VectorXf X_copy = currentState.replicate(1, 1);

    // calculate A matrix
    for(int i = 0; i < NUM_STATES; i++){
        // Create an incremented and decremented version of current state
        VectorXf X_inc = X_copy.replicate(1,1);
        X_inc(i) += epsState[i];
        VectorXf X_dec = X_copy.replicate(1, 1);
        X_dec(i) -= epsState[i];

        // apply same controls and observe how state variables change with respect to changing individual state variables
        ArrayXf stateInc(NUM_STATES);
        ArrayXf stateDec(NUM_STATES);
        ArrayXf _inc(NUM_STATES);
        ArrayXf _dec(NUM_STATES);
//        startTimerLoading = high_resolution_clock::now();

        globalMujocoController->loadSimulationState(baseStateIndex);
        globalMujocoController->setSystemState(X_inc);

//        stopTimerLoading = high_resolution_clock::now();
//        loadingDuration = duration_cast<microseconds>(stopTimerLoading - startTimerLoading);
//        microSecs_Loading += loadingDuration.count();
        stepSimulation(X_inc, currentControls, _inc, stateInc, numberMujocoStepsNeeded);
//        startTimerLoading = high_resolution_clock::now();

        globalMujocoController->loadSimulationState(baseStateIndex);
        globalMujocoController->setSystemState(X_dec);

//        stopTimerLoading = high_resolution_clock::now();
//        loadingDuration = duration_cast<microseconds>(stopTimerLoading - startTimerLoading);
//        microSecs_Loading += loadingDuration.count();
        stepSimulation(X_dec, currentControls, _dec, stateDec, numberMujocoStepsNeeded);

        // calculate the gradient
        for(int j = 0; j < NUM_STATES; j++){
            // CHECK, needs double checking especially with mujoco
            A(j, i) = (stateInc(j) - stateDec(j) ) / (2 * epsState[i]);
        }
    }

    VectorXf U_copy = currentControls.replicate(1, 1);
    for(int i = 0; i < DOF; i++){
        VectorXf U_inc = U_copy.replicate(1, 1);
        VectorXf U_dec = U_copy.replicate(1, 1);

        U_inc(i) += epsControls[i];
        U_dec(i) -= epsControls[i];

        ArrayXf stateInc(NUM_STATES);
        ArrayXf stateDec(NUM_STATES);
        ArrayXf _inc(NUM_STATES);
        ArrayXf _dec(NUM_STATES);

//        startTimerLoading = high_resolution_clock::now();

        globalMujocoController->loadSimulationState(baseStateIndex);

//        stopTimerLoading = high_resolution_clock::now();
//        loadingDuration = duration_cast<microseconds>(stopTimerLoading - startTimerLoading);
//        microSecs_Loading += loadingDuration.count();
        stepSimulation(X_copy, U_inc, _inc, stateInc, numberMujocoStepsNeeded);

//        startTimerLoading = high_resolution_clock::now();

        globalMujocoController->loadSimulationState(baseStateIndex);
        //globalMujocoController->setSystemState(X_copy, accels);

//        stopTimerLoading = high_resolution_clock::now();
//        loadingDuration = duration_cast<microseconds>(stopTimerLoading - startTimerLoading);
//        microSecs_Loading += loadingDuration.count();

        stepSimulation(X_copy, U_dec, _dec, stateDec, numberMujocoStepsNeeded);

        for(int j = 0; j < NUM_STATES; j++){
            B(j, i) = (stateInc(j) - stateDec(j))/(2 * epsControls[i]);
        }
    }
    globalMujocoController->loadSimulationState(baseStateIndex);

//    auto linDynStop = high_resolution_clock::now();
//    auto linDynDur = duration_cast<microseconds>(linDynStop - linDynStart);

//    cout << "Time taken by linearising dynamics: " << linDynDur.count() << " microseconds" << endl;
//    cout << "Time taken loading was " << microSecs_Loading << endl;
}

void stepSimulation(const Ref<m_state> currentState, const Ref<m_dof> U, Ref<m_state> Xnew, Ref<m_state> Xdot, int numSimSteps){

    globalMujocoController->setNextControlSequence(U);
    for(int i = 0; i < numSimSteps; i++){
        globalMujocoController->step();
    }

    Xnew = globalMujocoController->returnSystemState();

    for(int i = 0; i < NUM_STATES; i++) {
        Xdot(i) = (Xnew(i) - currentState(i)) / (MUJOCO_TIMESTEP * numSimSteps);
    }
}

float rollOutTrajectory(const Ref<const VectorXf> X0, m_state *X, m_dof *U, int numControls){

    float cost = 0.0f;
    float l;
    globalMujocoController->loadSimulationState(initStateIndex);
    X[0] = X0;
    m_state l_x;
    m_state_state l_xx;
    m_dof l_u;
    m_dof_dof l_uu;

    for(int i = 0; i < numControls; i++){
        // Calculate cost associated with current state Xt and current control Ut

//        cout << "Current control in rollout, iteration: " << i << " " << U[i] << endl;
//        cout << "Current state in rollout, iteration: " << i << " " << X[i] << endl;
        l = immediateCost(X[i], X[i+1], U[i]);
        cost += (l * dt);

        // Step simulation set number of times
        globalMujocoController->setNextControlSequence(U[i]);
        for(int j = 0; j < mujoco_steps_per_dt; j++){
            globalMujocoController->step();
        }


        // update new state variable Xt+1
        X[i+1] = globalMujocoController->returnSystemState();

    }
    return cost;
}

float immediateCost(const Ref<const m_state> X, const Ref<const m_state> X_next, const Ref<const m_dof> U){
    float cost;
    float eps = 1e-1;
    m_state X_diff;

    // actual - desired
    X_diff = X - X_desired;

    cost = calcStateCost(X, X_next, false) + calcControlCost(U);

    return cost;
}

float immediateCostAndDerivitives(Ref<VectorXf> l_x, Ref<MatrixXf> l_xx, Ref<VectorXf> l_u, Ref<MatrixXf> l_uu, const Ref<const VectorXf> X, const Ref<const VectorXf> X_next, const Ref<const VectorXf> U){
    float cost;
    float eps = 1e-1;
    m_state X_diff;

    // actual - desired
    X_diff = X - X_desired;

    cost = calcStateCost(X, X_next, false) + calcControlCost(U);

    if(costFunctionFD){
        l_x = costFirstOrderDerivitives(X, X_next, false);
        for(int i = 0; i < NUM_STATES; i++){
            m_state X_inc = X.replicate(1, 1);
            m_state X_dec = X.replicate(1, 1);

            X_inc(i) += eps;
            X_dec(i) -= eps;

            m_state l_x_inc = costFirstOrderDerivitives(X_inc, X_next, false);
//        cout << "l_x_inc" << endl;
//        cout << l_x_inc << endl;
            m_state l_x_dec = costFirstOrderDerivitives(X_dec, X_next, false);
//        cout << "l_x_dec" << endl;
//        cout << l_x_dec << endl;

            for(int j = 0 ; j < NUM_STATES; j++){
                l_xx(j, i) = (l_x_inc(j) - l_x_dec(j)) / (2 * eps);
            }
        }
    }
    else{
        l_x = Q * X_diff;
        l_xx = Q;
    }




//    cout << "X_diff was" << endl;
//    cout << X_diff << endl;
//
//    m_state orig_l_x = (Q * X_diff);
//    m_state_state orig_l_xx = Q;
//    cout << "original l_x" << endl;
//    cout << orig_l_x << endl;
//
//    cout << "new l_x" << endl;
//    cout << l_x << endl;
//
//    cout << "original l_xx" << endl;
//    cout << orig_l_xx << endl;
//
//    cout << "new l_xx" << endl;
//    cout << l_xx << endl;

    l_u = R * U;
    l_uu = R;

    return cost;
}

float terminalCost(Ref<VectorXf> l_x, Ref<MatrixXf> l_xx, const Ref<const VectorXf> X){
    float cost;
    float eps = 1e-1;

    cost = calcStateCost(X, X, true);

    l_x = costFirstOrderDerivitives(X, X, true);

    for(int i = 0; i < NUM_STATES; i++){
        m_state X_inc = X.replicate(1, 1);
        m_state X_dec = X.replicate(1, 1);

        X_inc(i) += eps;
        X_dec(i) -= eps;

        m_state l_x_inc = costFirstOrderDerivitives(X_inc, X_inc, true);
        m_state l_x_dec = costFirstOrderDerivitives(X_dec, X_inc, true);

        for(int j = 0 ; j < NUM_STATES; j++){
            l_xx(j, i) = (l_x_inc(j) - l_x_dec(j)) / (2 * eps);
        }
    }

    return cost;
}


m_state costFirstOrderDerivitives(const Ref<const m_state> X, m_state X_next, bool terminal){
    m_state l_x;
    m_state X_inc;
    m_state X_dec;
    float eps = 1e-2;

    for(int i = 0; i < NUM_STATES; i++){
        X_inc = X.replicate(1, 1);
        X_dec = X.replicate(1, 1);

        X_inc(i) += eps;
        X_dec(i) -= eps;

        float incCost = calcStateCost(X_inc, X_next, terminal);
        float decCost = calcStateCost(X_dec, X_next, terminal);

        l_x(i) = (incCost - decCost) / (2 * eps);
    }


    return l_x;
}

float calcStateCost(const Ref<const m_state> X,const Ref<const m_state> X_next, bool terminal){
    float stateCost;
    m_state X_diff;
    m_dof accel;
    VectorXf temp(1);
    VectorXf tempAccel(1);

    // actual - desired
    X_diff = X - X_desired;

    if(terminal){
        temp = 0.5 * X_diff.transpose() * Q_term * X_diff;
    }
    else{
        temp = 0.5 * X_diff.transpose() * Q * X_diff;
    }

//    for(int i = 0; i < DOF; i++){
//        accel(i) = (X_next(i + 7) - X(i + 7)) / dt;
//    }

    //tempAccel = accel.transpose() * Z * accel;
    stateCost = temp(0); //+ tempAccel(0);


    return stateCost;
}

float calcControlCost(const Ref<const m_dof> U){
    float controlCost;
    VectorXf temp(1);

    temp = 0.5 * U.transpose() * R * U;
    controlCost = temp(0);

    return controlCost;
}

void warmStartControls(m_dof *U, Ref<m_state> X0){
    m_state stateDiff = X0 - X_desired;

    for(int i = 0; i < DOF - 3; i++){
        float startTorque = stateDiff(i) * 40;
        if(startTorque > torqueLims[i]) startTorque = torqueLims[i];
        if(startTorque < -torqueLims[i]) startTorque = -torqueLims[i];
        float torqueChangePerControl = startTorque / numControls;
        U[0](i) = startTorque;
        float currentTorque = startTorque;
        for(int j = 0; j < numControls; j++){
            U[j](i) = currentTorque;
            currentTorque = currentTorque - torqueChangePerControl;
        }
    }

    for(int i = DOF - 3; i < DOF ; i++){
        float startTorque = stateDiff(i) * 5;
        if(startTorque > torqueLims[i]) startTorque = torqueLims[i];
        if(startTorque < -torqueLims[i]) startTorque = -torqueLims[i];
        float torqueChangePerControl = startTorque / numControls;
        U[0](i) = startTorque;
        float currentTorque = startTorque;
        for(int j = 0; j < numControls; j++){
            U[j](i) = currentTorque;
            currentTorque = currentTorque - torqueChangePerControl;
        }
    }
}

void initCostMatrices(){

    R.setIdentity();
    for(int i = 0; i < DOF; i++){
        R(i, i) = controlCost[i];
    }
    Q.setIdentity();
    for(int i = 0; i < NUM_STATES; i++){
        Q(i, i) = stateCosts[i];
    }
    Q_term = Q.replicate(1, 1);
    for(int i = 0; i < DOF; i++){
        Q_term(i, i) *= termStateCosts[i];
    }
    Z.setIdentity();
    for(int i = 0; i < DOF; i++){
        Z(i, i) = accelCosts[i];
    }
}

void initDesiredState(){
    X_desired << 0, 0, 0, -0.4, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0,
                 0.7, 0;

    // Currently its the last two entries in the desired state that matter, which is the x and y position of the cube

    outputFile.open(filename);
    outputFile << "Control Number" << "," << "T1" << "," << "T2" << "," << "T3" << "," << "T4" << "," << "T5" << "," << "T6" << "," << "T7" << ",";
    outputFile << "V1" << "," << "V2" << "," << "V3" << "," << "V4" << "," << "V5" << "," << "V6" << "," << "V7" << ",";
    outputFile << "p1" << "," << "p2" << "," << "p3" << "," << "p4" << "," << "p5" << "," << "p6" << "," << "p7" << endl;

}

void saveTrajecToCSV(m_dof *U, m_state *X){
    for(int i = 0; i < numControls; i++){
        outputFile << i << "," << U[i](0) << "," << U[i](1) << "," << U[i](2) << "," << U[i](3) << "," << U[i](4) << "," << U[i](5) << "," << U[i](6) << ",";

        outputFile << X[i](7);
        for(int j = 0; j < DOF - 1; j++){
            outputFile << "," << X[i](j+1+7);
        }

        outputFile << "," << X[i](0);
        for(int j = 0; j < DOF - 1; j++){
            outputFile << "," << X[i](j+1);
        }
        outputFile << endl;
    }
    outputFile.close();
}