#ifndef OSQP_INTERFACE_H
#define OSQP_INTERFACE_H
#include <osqp/osqp.h>
#include <Eigen/Sparse>
#include <memory>
/**
 * osqp interface:
 * minimize     0.5 x' P x + q' x
 * subject to   l <= A x <= u
 **/

namespace osqp {
class OSQPInterface{
private:
  // Exitflag
  c_int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings  *settings;
  OSQPData      *data;

public:
  OSQPInterface();
  ~OSQPInterface();
  int updateMatrices(
    Eigen::SparseMatrix<double> & P_, 
    Eigen::SparseMatrix<double> & q_, 
    Eigen::SparseMatrix<double> & A_, 
    Eigen::SparseMatrix<double> & l_, 
    Eigen::SparseMatrix<double> & u_,
    int warmStart = WARM_START
  );
  int solveQP();
  int solveStatus();
  OSQPSolution* solPtr();
  typedef std::shared_ptr<OSQPInterface> Ptr;
};
} // namespace osqp

#endif