#include "osqp_interface/osqp_interface.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace osqp{
OSQPInterface::OSQPInterface(){
    settings = new OSQPSettings;
    data     = new OSQPData;
}
OSQPInterface::~OSQPInterface(){
    delete settings;
    delete data;
    osqp_cleanup(work);
}
int OSQPInterface::updateMatrices(
    Eigen::SparseMatrix<double> & P_, 
    Eigen::SparseMatrix<double> & q_, 
    Eigen::SparseMatrix<double> & A_, 
    Eigen::SparseMatrix<double> & l_, 
    Eigen::SparseMatrix<double> & u_,
    int warmStart
){
    // change Q_ upper triangular
    P_ = P_.triangularView<Upper>();
    // compress the matrices for osqp data
    P_.makeCompressed();
    q_.makeCompressed();
    A_.makeCompressed();
    l_.makeCompressed();
    u_.makeCompressed();

    data->n = P_.rows();
    data->m = A_.rows();
    // use the eigen sparse ptrs make it faster
    data->P = csc_matrix(data->n, data->n, P_.nonZeros(), 
        P_.valuePtr(), P_.innerIndexPtr(), P_.outerIndexPtr());
    data->q = new c_float[q_.rows()];
    for (int i=0; i<q_.rows(); ++i){
        data->q[i] = q_.coeff(i,0);
    }
    data->A = csc_matrix(data->m, data->n, A_.nonZeros(), 
        A_.valuePtr(), A_.innerIndexPtr(), A_.outerIndexPtr());
    data->l = new c_float[l_.rows()];
    for (int i=0; i<l_.rows(); ++i){
        data->l[i] = l_.coeff(i,0);
    }
    data->u = new c_float[u_.rows()];
    for (int i=0; i<u_.rows(); ++i){
        data->u[i] = u_.coeff(i,0);
    }
    if (settings) osqp_set_default_settings(settings);
    settings->eps_abs = 1e-3;
    settings->eps_rel = 1e-3;
    settings->max_iter = 1e4;
    settings->warm_start = warmStart;
    exitflag = osqp_setup(&work, data, settings);
    delete data->q;
    delete data->l;
    delete data->u;
    return 0;
}
int OSQPInterface::solveQP(){
    // Solve Problem
    return osqp_solve(work);
}
int OSQPInterface::solveStatus(){
    return work->info->status_val;
}
OSQPSolution* OSQPInterface::solPtr(){
    return work->solution;
}
}// namespace osqp