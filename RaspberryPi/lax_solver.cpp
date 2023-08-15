#include "lax_solver.h"
#include <math.h>
#include <string.h>

/**
 * Sparse ADMM solver for the lax MPC formulation
 *
 * ARGUMENTS:
 * The current system state is given in "pointer_x0". Pointer to array of size nn.
 * The state reference is given in "pointer_xr". Pointer to array of size nn.
 * The input reference is given in "pointer_ur". Pointer to array of size mm.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm.
 * The number of iterations is returned in "pointer_k". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 *
 * If CONF_MATLAB is defined, them the solver uses slightly different arguments for the mex file.
 *
 */

#include <stdio.h>
#ifdef MEASURE_TIME
#include <time.h>
#endif

double Ad[] = {1.0132 , 0.0201 , 1.3181 , 1.0132};
double Bd[] = {-0.0001, -0.0144};

double x1 = -0.5;
double x2 = 0;
double dx[] = {0 , 0};
double u = 0;

double* xr[] = {0 , 0};
double* ur[] = {0};
double* u_opt[] = {0};
int* pointer_k[] = {0};
int* e_flag[] = {0};
solution* sol;

int main() {
    dx[1] = Ad[1]*x1 + Ad[2]*x2 + Bd[1]*u;
    dx[2] = Ad[3]*x1 + Ad[4]*x2 + Bd[2]*u;
    
    x1 = dx[1];
    x2 = dx[2];
    
    double *x_ptr1 = &x1;
    
    laxMPC_ADMM(x_ptr1, *xr, *ur, *u_opt, *pointer_k, *e_flag,  sol);
    return 1;
}


#ifdef CONF_MATLAB

void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt, double *update_time, double *solve_time, double *polish_time, double *run_time){

#else

void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, solution *sol){

#endif

    #ifdef MEASURE_TIME
    // Initialize time variables
    struct timespec start, post_update, post_solve, post_polish;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    #endif

    // Initialize ADMM variables
    int done = 0; // Flag used to determine when the algorithm should exit
    #ifdef CONF_MATLAB
    double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
    #else
    int k = 0; // Number of iterations
    #endif
    double x0[nn]; // Current system state
    double xr[nn]; // State reference
    double ur[mm]; // Control input reference
    double v[NN-1][nm] = {{0.0}}; // Decision variables v
    double v_0[mm] = {0.0};
    double v_N[nn] = {0.0};
    double lambda[NN-1][nm] = {{0.0}}; // Dual variables lambda
    double lambda_0[mm] = {0.0};
    double lambda_N[nn] = {0.0};
    double z[NN-1][nm] = {{0.0}}; // Decision variables z
    double z_0[mm] = {0.0};
    double z_N[nn] = {0.0};
    double v1[NN-1][nm] = {{0.0}}; // Value of the decision variables z at the last iteration
    double v1_0[mm] = {0.0};
    double v1_N[nn] = {0.0};
    double aux_N[nn] = {0.0}; // Auxiliary array used for multiple purposes
    double mu[NN][nn] = {{0.0}}; // Used to solve the system of equations
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
    double res_fixed_point; // Variable used to determine if a fixed point has been reached
    double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
    double b[nn] = {0.0}; // First nn components of vector b (the rest are known to be zero)
    double q[nm] = {0.0};
    double qT[nn] = {0.0};
    
    // Constant variables
    const static double LB[3] = { -1.570796326794897, -4.000000000000000, -50.000000000000000 };
const static double UB[3] = { 1.570796326794897, 4.000000000000000, 50.000000000000000 };
const static double Hi[9][3] = { {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914}, {0.040000000000000, 0.062500000000000, 0.066225165562914} };
const static double Hi_0[1] = { 0.066225165562914 };
const static double Hi_N[2][2] = { {0.003689783146828, -0.007915667568827}, {-0.007915667568827, 0.030995295768202} };
const static double AB[2][3] = { {1.001422076052727, 0.020009479608522, -0.000151207534663}, {0.142241300695365, 1.001422076052727, -0.015124336257652} };
const static double Alpha[9][2][2] = { {{-0.200284411419779, -0.028448259600635}, {-0.005001157183762, -0.250325097859900}}, {{-0.200284377246451, -0.028448254746679}, {-0.004998722030993, -0.250294349235162}}, {{-0.200284282011329, -0.028448241219548}, {-0.004995042720402, -0.250263111225424}}, {{-0.200284094449693, -0.028448214578423}, {-0.004990084969764, -0.250231217396007}}, {{-0.200283782421237, -0.028448170258116}, {-0.004983801694876, -0.250198495340347}}, {{-0.200283312617067, -0.028448103527456}, {-0.004976132683116, -0.250164765213593}}, {{-0.200282650263328, -0.028448009447188}, {-0.004967004187407, -0.250129838250943}}, {{-0.200281758821293, -0.028447882827376}, {-0.004956328445723, -0.250093515273086}}, {{-0.200280599684139, -0.028447718184335}, {-0.004944003133073, -0.250055585183730}} };
const static double Beta[10][2][2] = { {{4.999999905365407, 0.000000757256144}, {0.000000000000000, 3.999515329346603}}, {{4.999999052245419, 0.000003037092519}, {0.000000000000000, 3.999028193789424}}, {{4.999996674748341, 0.000006863228662}, {0.000000000000000, 3.998536048467200}}, {{4.999991992366150, 0.000012275035507}, {0.000000000000000, 3.998036304659702}}, {{4.999984202732209, 0.000019327827152}, {0.000000000000000, 3.997526307096193}}, {{4.999972474306666, 0.000028093258821}, {0.000000000000000, 3.997003311202572}}, {{4.999955938977687, 0.000038659827277}, {0.000000000000000, 3.996464460285050}}, {{4.999933684574268, 0.000051133468305}, {0.000000000000000, 3.995906762676879}}, {{4.999904747296430, 0.000065638243728}, {0.000000000000000, 3.995327068911679}}, {{16.457937697781585, -0.130004602569566}, {0.000000000000000, 8.374325714470592}} };
const static double Q[2] = { -10.000000000000000, -1.000000000000000 };
const static double R[1] = { -0.100000000000000 };
const static double T[2][2] = { {-584.427935774591447, -153.083626191034682}, {-153.083626191034682, -56.357896104603995} };


    // Obtain variables in scaled units
    #if in_engineering == 1
    for(unsigned int i = 0; i < nn; i++){
        x0[i] = scaling_x[i]*( pointer_x0[i] - OpPoint_x[i] );
        xr[i] = scaling_x[i]*( pointer_xr[i] - OpPoint_x[i] );
    }
    for(unsigned int i = 0; i < mm; i++){
        ur[i] = scaling_u[i]*( pointer_ur[i] - OpPoint_u[i] );
    }
    #endif
    #if in_engineering == 0
    for(unsigned int i = 0; i < nn; i++){
        x0[i] = pointer_x0[i];
        xr[i] = pointer_xr[i];
    }
    for(unsigned int i = 0; i < mm; i++){
        ur[i] = pointer_ur[i];
    }
    #endif
 
    // Update first nn elements of beq
    for(unsigned int j = 0; j < nn; j++){
        b[j] = 0.0;
        for(unsigned int i = 0; i < nn; i++){
            b[j] = b[j] - AB[j][i]*x0[i];
        }
    }

    // Update the reference
    for(unsigned int j = 0; j < nn; j++){
        q[j] = Q[j]*xr[j];
        qT[j] = 0.0;
        for(unsigned int i = 0; i < nn; i++){
            qT[j] = qT[j] + T[j][i]*xr[i];
        }
    }
    for(unsigned int j = 0; j < mm; j++){
        q[j+nn] = R[j]*ur[j];
    }

    // Measure time
    #ifdef MEASURE_TIME
    clock_gettime(CLOCK_MONOTONIC_RAW, &post_update);
    #ifdef CONF_MATLAB
    *update_time = (double) ( (post_update.tv_sec - start.tv_sec) * 1000.0 ) + (double) ( (post_update.tv_nsec - start.tv_nsec) / 1000000.0 );
    #else
    sol->update_time = (double) ( (post_update.tv_sec - start.tv_sec) * 1000.0 ) + (double) ( (post_update.tv_nsec - start.tv_nsec) / 1000000.0 );
    #endif
    #endif

    // Algorithm
    while(done == 0){

        k += 1; // Increment iteration counter

        // Step 0: Save the value of v into variable v1
        memcpy(v1_0, v_0, sizeof(double)*mm);
        memcpy(v1, v, sizeof(double)*(NN-1)*nm);
        memcpy(v1_N, v_N, sizeof(double)*nn);

        // Step 1: Minimize w.r.t. z

        // Compute vector q_hat = lambda - rho*v
        // I store vector q_hat in z because I save memory and computation
        
        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
            #ifdef SCALAR_RHO
            z_0[j] = q[j+nn] + lambda_0[j] - rho*v_0[j];
            #else
            z_0[j] = q[j+nn] + lambda_0[j] - rho_0[j]*v_0[j];
            #endif
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nm; j++){
                #ifdef SCALAR_RHO
                z[l][j] = q[j] + lambda[l][j] - rho*v[l][j];
                #else
                z[l][j] = q[j] + lambda[l][j] - rho[l][j]*v[l][j];
                #endif
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            #ifdef SCALAR_RHO
            z_N[j] = qT[j] + lambda_N[j] - rho*v_N[j];
            #else
            z_N[j] = qT[j] + lambda_N[j] - rho_N[j]*v_N[j];
            #endif
        }

        // Compute r.h.s of the Wc system of equations, i.e., -G'*H_hat^(-1)*q_hat - b
        // I store it in mu to save a bit of memory

        // Compute the first nn elements
        for(unsigned int j = 0; j < nn; j++){
            mu[0][j] = Hi[0][j]*z[0][j] - b[j];
            for(unsigned int i = 0; i < mm; i++){
                mu[0][j] = mu[0][j] - AB[j][i+nn]*Hi_0[i]*z_0[i];
            }
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 1; l < NN-1; l++){
            for(unsigned int j = 0; j < nn; j++){
                mu[l][j] = Hi[l][j]*z[l][j];
                for(unsigned int i = 0; i < nm; i++){
                    mu[l][j] = mu[l][j] - AB[j][i]*Hi[l-1][i]*z[l-1][i];
                }
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            mu[NN-1][j] = 0.0;
            for(unsigned int i = 0; i < nn; i++){
                mu[NN-1][j] = mu[NN-1][j] + Hi_N[j][i]*z_N[i];
            }
            for(unsigned int i = 0; i < nm; i++){
                mu[NN-1][j] = mu[NN-1][j] - AB[j][i]*Hi[NN-2][i]*z[NN-2][i];
            }
        }
        
        // Compute mu, the solution of the system of equations W*mu = -G'*H^(-1)*q_hat - beq

        // FORWARD SUBSTITUTION

        // Compute first nn elements
        for(unsigned int j = 0; j < nn; j++){
            for(unsigned int i = 0; i < j; i++){
                mu[0][j] = mu[0][j] - Beta[0][i][j]*mu[0][i];
            }
            mu[0][j] = Beta[0][j][j]*mu[0][j];
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 1; l < NN-1; l++){
            for(unsigned int j = 0; j < nn; j++){
                for(unsigned int i = 0; i < nn; i++){
                    mu[l][j] = mu[l][j] - Alpha[l-1][i][j]*mu[l-1][i];
                }
                for(unsigned int i = 0; i < j; i++){
                    mu[l][j] = mu[l][j] - Beta[l][i][j]*mu[l][i];
                }
                mu[l][j] = Beta[l][j][j]*mu[l][j];
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            for(unsigned int i = 0; i < nn; i++){
                mu[NN-1][j] = mu[NN-1][j] - Alpha[NN-2][i][j]*mu[NN-2][i];
            }
            for(unsigned int i = 0; i < j; i++){
                mu[NN-1][j] = mu[NN-1][j] - Beta[NN-1][i][j]*mu[NN-1][i];
            }
            mu[NN-1][j] = Beta[NN-1][j][j]*mu[NN-1][j];
        }

        // BACKWARD SUBSTITUTION

        // Compute the last nn elements
        for(unsigned int j = nn-1; j != -1; j--){
            for(unsigned int i = nn-1; i >= j+1; i--){
                mu[NN-1][j] = mu[NN-1][j] - Beta[NN-1][j][i]*mu[NN-1][i];
            }
            mu[NN-1][j] = Beta[NN-1][j][j]*mu[NN-1][j];
        }

        // Compute all the other elements except for the first nn
        for(unsigned int l = NN-2; l >=1; l--){
            for(unsigned int j = nn-1; j != -1; j--){
                for(unsigned int i = nn-1; i != -1; i--){
                    mu[l][j] = mu[l][j] - Alpha[l][j][i]*mu[l+1][i];
                }
                for(unsigned int i = nn-1; i >= j+1; i--){
                    mu[l][j] = mu[l][j] - Beta[l][j][i]*mu[l][i];
                }
                mu[l][j] = Beta[l][j][j]*mu[l][j];
            }
        }

        // Compute the first nn elements
        for(unsigned int j = nn-1; j != -1; j--){
            for(unsigned int i = nn-1; i != -1; i--){
                mu[0][j] = mu[0][j] - Alpha[0][j][i]*mu[1][i];
            }
            for(unsigned int i = nn-1; i >= j+1; i--){
                mu[0][j] = mu[0][j] - Beta[0][j][i]*mu[0][i];
            }
            mu[0][j] = Beta[0][j][j]*mu[0][j];
        }

        // Compute z (note that, from before, we have that at this point z = q_hat)

        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
            for(unsigned int i = 0; i < nn; i++){
                z_0[j] = z_0[j] + AB[i][j+nn]*mu[0][i];
            }
            z_0[j] = -Hi_0[j]*z_0[j];
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nn; j++){
                z[l][j] = z[l][j] - mu[l][j];
            }
            for(unsigned int j = 0; j < nm; j++){
                for(unsigned int i = 0; i < nn; i++){
                    z[l][j] = z[l][j] + AB[i][j]*mu[l+1][i];
                }
                z[l][j] = -Hi[l][j]*z[l][j];
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            aux_N[j] = z_N[j] - mu[NN-1][j];
        }
        for(unsigned int j = 0; j < nn; j++){
            z_N[j] = 0.0;
            for(unsigned int i = 0; i < nn; i++){
                z_N[j] = z_N[j] - Hi_N[j][i]*aux_N[i];
            }
        }

        // Step 2: Minimize w.r.t. v

        // Compute the first mm variables
        for(unsigned int j = 0; j < mm; j++){
            #ifdef SCALAR_RHO
            v_0[j] = z_0[j] + rho_i*lambda_0[j];
            #else
            v_0[j] = z_0[j] + rho_i_0[j]*lambda_0[j];
            #endif
            #ifdef VAR_BOUNDS
            v_0[j] = (v_0[j] > LB0[j]) ? v_0[j] : LB0[j]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UB0[j]) ? UB0[j] : v_0[j]; // minimum between v and the upper bound
            #else
            v_0[j] = (v_0[j] > LB[j+nn]) ? v_0[j] : LB[j+nn]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UB[j+nn]) ? UB[j+nn] : v_0[j]; // minimum between v and the upper bound
            #endif
        }

        // Compute all the other elements except the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nm; j++){
                #ifdef SCALAR_RHO
                v[l][j] = z[l][j] + rho_i*lambda[l][j];
                #else
                v[l][j] = z[l][j] + rho_i[l][j]*lambda[l][j];
                #endif
                #ifdef VAR_BOUNDS
                v[l][j] = (v[l][j] > LB[l][j]) ? v[l][j] : LB[l][j]; // maximum between v and the lower bound
                v[l][j] = (v[l][j] > UB[l][j]) ? UB[l][j] : v[l][j]; // minimum between v and the upper bound
                #else
                v[l][j] = (v[l][j] > LB[j]) ? v[l][j] : LB[j]; // maximum between v and the lower bound
                v[l][j] = (v[l][j] > UB[j]) ? UB[j] : v[l][j]; // minimum between v and the upper bound
                #endif
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            #ifdef SCALAR_RHO
            v_N[j] = z_N[j] + rho_i*lambda_N[j];
            #else
            v_N[j] = z_N[j] + rho_i_N[j]*lambda_N[j];
            #endif
            #ifdef VAR_BOUNDS
            v_N[j] = (v_N[j] > LBN[j]) ? v_N[j] : LBN[j]; // maximum between v and the lower bound
            v_N[j] = (v_N[j] > UBN[j]) ? UBN[j] : v_N[j]; // minimum between v and the upper bound
            #else
            v_N[j] = (v_N[j] > LB[j]) ? v_N[j] : LB[j]; // maximum between v and the lower bound
            v_N[j] = (v_N[j] > UB[j]) ? UB[j] : v_N[j]; // minimum between v and the upper bound
            #endif
        }

        // Step 3: Update lambda

        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
            #ifdef SCALAR_RHO
            lambda_0[j] = lambda_0[j] + rho*( z_0[j] - v_0[j] );
            #else
            lambda_0[j] = lambda_0[j] + rho_0[j]*( z_0[j] - v_0[j] );
            #endif
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nm; j++){
                #ifdef SCALAR_RHO
                lambda[l][j] = lambda[l][j] + rho*( z[l][j] - v[l][j] );
                #else
                lambda[l][j] = lambda[l][j] + rho[l][j]*( z[l][j] - v[l][j] );
                #endif
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            #ifdef SCALAR_RHO
            lambda_N[j] = lambda_N[j] + rho*( z_N[j] - v_N[j] );
            #else
            lambda_N[j] = lambda_N[j] + rho_N[j]*( z_N[j] - v_N[j] );
            #endif
        }

        // Step 4: Compute the residual

        res_flag = 0; // Reset the residual flag

        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
            res_fixed_point = v1_0[j] - v_0[j];
            res_primal_feas = z_0[j] - v_0[j];
            // Obtain absolute values
            res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
            res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
            if( res_fixed_point > tol || res_primal_feas > tol){
                res_flag = 1;
                break;
            }
        }

        // Compute the last NN elements
        if(res_flag == 0){
            for(unsigned int j = 0; j < nn; j++){
                res_fixed_point = v1_N[j] - v_N[j];
                res_primal_feas = z_N[j] - v_N[j];
                // Obtain absolute values
                res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
                res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
                if( res_fixed_point > tol || res_primal_feas > tol){
                    res_flag = 1;
                    break;
                }
            }
        }

        // Compute all the other elements
        if(res_flag == 0){
            for(unsigned int l = 0; l < NN-1; l++){
                for(unsigned int j = 0; j < nm; j++){
                    res_fixed_point = v1[l][j] - v[l][j];
                    res_primal_feas = z[l][j] - v[l][j];
                    // Obtain absolute values
                    res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
                    res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
                    if( res_fixed_point > tol || res_primal_feas > tol){
                        res_flag = 1;
                        break;
                    }
                }
                if(res_flag == 1){
                    break;
                }
            }
        }

        // Step 5: Exit condition

        if(res_flag == 0){
            done = 1;
            #ifdef CONF_MATLAB
            e_flag[0] = 1.0;
            #else
            *e_flag = 1;
            #endif
        }
        else if( k >= k_max ){
            done = 1;
            #ifdef CONF_MATLAB
            e_flag[0] = -1.0;
            #else
            *e_flag = -1;
            #endif
        }

    }

    // Measure time
    #ifdef MEASURE_TIME
    clock_gettime(CLOCK_MONOTONIC_RAW, &post_solve);
    #ifdef CONF_MATLAB
    *solve_time = (double) ( (post_solve.tv_sec - post_update.tv_sec) * 1000.0 ) + (double) ( (post_solve.tv_nsec - post_update.tv_nsec) / 1000000.0 );
    #else
    sol->solve_time = (double) ( (post_solve.tv_sec - post_update.tv_sec) * 1000.0 ) + (double) ( (post_solve.tv_nsec - post_update.tv_nsec) / 1000000.0 );
    #endif
    #endif

    // Control action
    #if in_engineering == 1
    for(unsigned int j = 0; j < mm; j++){
        u_opt[j] = v_0[j]*scaling_i_u[j] + OpPoint_u[j];
    }
    #endif
    #if in_engineering == 0
    for(unsigned int j = 0; j < mm; j++){
        u_opt[j] = v_0[j];
    }
    #endif

    // Return number of iterations
    #ifdef CONF_MATLAB
    pointer_k[0] = k; // Number of iterations
    #else
    *pointer_k = k; // Number of iterations
    #endif

    // Save solution into structure
    #ifdef DEBUG

    #ifdef CONF_MATLAB

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm; j++){
        count++;
        z_opt[count] = z_0[j];
        v_opt[count] = v_0[j];
        lambda_opt[count] = lambda_0[j];
    }

    // All other elements except the last nn
    for(unsigned int l = 0; l < NN-1; l++){
        for(unsigned int j = 0; j < nm; j++){
            count++;
            z_opt[count] = z[l][j];
            v_opt[count] = v[l][j];
            lambda_opt[count] = lambda[l][j];
        }
    }

    // Last nn elements
    for(unsigned int j = 0; j < nn; j++){
        count++;
        z_opt[count] = z_N[j];
        v_opt[count] = v_N[j];
        lambda_opt[count] = lambda_N[j];
    }

    #else

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm; j++){
        count++;
        sol->z[count] = z_0[j];
        sol->v[count] = v_0[j];
        sol->lambda[count] = lambda_0[j];
    }

    // All other elements except the last nn
    for(unsigned int l = 0; l < NN-1; l++){
        for(unsigned int j = 0; j < nm; j++){
            count++;
            sol->z[count] = z[l][j];
            sol->v[count] = v[l][j];
            sol->lambda[count] = lambda[l][j];
        }
    }

    // Last nn elements
    for(unsigned int j = 0; j < nn; j++){
        count++;
        sol->z[count] = z_N[j];
        sol->v[count] = v_N[j];
        sol->lambda[count] = lambda_N[j];
    }

    #endif

    #endif

    // Measure time
    #ifdef MEASURE_TIME
    clock_gettime(CLOCK_MONOTONIC_RAW, &post_polish);
    #ifdef CONF_MATLAB
    *run_time = (double) ( (post_polish.tv_sec - start.tv_sec) * 1000.0 ) + (double) ( (post_polish.tv_nsec - start.tv_nsec) / 1000000.0 );
    *polish_time = (double) ( (post_polish.tv_sec - post_solve.tv_sec) * 1000.0 ) + (double) ( (post_polish.tv_nsec - post_solve.tv_nsec) / 1000000.0 );
    #else
    sol->run_time = (double) ( (post_polish.tv_sec - start.tv_sec) * 1000.0 ) + (double) ( (post_polish.tv_nsec - start.tv_nsec) / 1000000.0 );
    sol->polish_time = (double) ( (post_polish.tv_sec - post_solve.tv_sec) * 1000.0 ) + (double) ( (post_polish.tv_nsec - post_solve.tv_nsec) / 1000000.0 );
    #endif
    #endif

}



// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies

// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies
