// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLP.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include <hpp/foot/solvers/NLP/MyNLP.hh>
#include <hpp/foot/utils/defs.hh>
#include <cassert>
#include <iostream>

using namespace Ipopt;
using namespace std;

/* Constructor. */
MyNLP::MyNLP()
{}

MyNLP::~MyNLP()
{}

bool MyNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // The problem described in MyNLP.hpp has 2 variables, x1, & x2,
  n = 108;

  // one equality constraint,
  m = 2;

  // 2 nonzeros in the jacobian (one for x1, and one for x2),
  nnz_jac_g = 2;

  // and 2 nonzeros in the hessian of the lagrangian
  // (one in the hessian of the objective for x2,
  //  and one in the hessian of the constraints for x1)
  nnz_h_lag = 108*108; // check it!

  // We use the standard fortran index style for row/col entries
  index_style = FORTRAN_STYLE;
  return true;
}

bool MyNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  assert(n == 108);
  assert(m == 2);

  // x1 has a lower bound of -1 and an upper bound of 1
  for (int i=0; i<n; i++){
    x_l[i] = -10.0;
    x_u[i] = 10.0;
  }

  // x2 has no upper or lower bound, so we set them to
  // a large negative and a large positive number.
  // The value that is interpretted as -/+infinity can be
  // set in the options, but it defaults to -/+1e19
//   x_l[1] = -1.0e19;
//   x_u[1] = +1.0e19;

  // we have one equality constraint, so we set the bounds on this constraint
  // to be equal (and zero).
  for (int i=0; i<2; i++)
      g_l[i] = g_u[i] = 0.0;
  
  return true;
}

bool MyNLP::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the others if
  // you wish.
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  // we initialize x in bounds, in the upper right quadrant
  for (int i=0; i<n; i++)
    x[i] = 2.0;
 return true;
}

bool MyNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  // return the value of the objective function 
  Eigen::Vector3d pos(-1.0, 0, 0.34);
  Eigen::Vector3d pos_next(x[0], x[1], x[2]);

  obj_value = std::pow((pos_next-pos).norm() ,2);
  for (int i=0; i<11; i++){
        pos = Eigen::Vector3d(x[3*i], x[3*i+1], x[3*i +2]);
        pos_next = Eigen::Vector3d(x[3*i+3], x[3*i+4], x[3*i+5]);
        obj_value += std::pow((pos_next-pos).norm() ,2);
  }

  return true;
}

bool MyNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  // return the gradient of the objective function grad_{x} f(x)
  // grad_{x1} f(x): x1 is not in the objective
  Eigen::Vector3d pos(-1.0, 0, 0.34);
  for (int i=0; i<3; i++){
    grad_f[i] = 4*x[i]- 2*pos(i) -2*x[3+i];
    grad_f[i+3] = 4*x[i+3] -2*x[i] -2*x[6+i];
    grad_f[i+6] = 4*x[i+6] -2*x[i+3] -2*x[9+i];
    grad_f[i+9] = 4*x[i+9] -2*x[i+6] -2*x[12+i];
    grad_f[i+12] = 2*x[i+12] -2*x[i+9];  
  }
  
  return true;
}

bool MyNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  // return the value of the constraints: g(x)
  for (int i=0; i<2; i++)
    g[i] = -(x[i]*x[i] - 1.0);


  return true;
}

bool MyNLP::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{
  if (values == NULL) {
     // return the structure of the jacobian of the constraints

    // element at 1,1: grad_{x1} g_{1}(x)
    for (int i=0; i<2; i++){
        iRow[i] = i+1;
        jCol[i] = i+1;
    }

    // element at 1,2: grad_{x2} g_{1}(x)
    //iRow[1] = 1;
    //jCol[1] = 2;
  }
  else {
    // return the values of the jacobian of the constraints
   Number x1 = x[0];

    // element at 1,1: grad_{x1} g_{1}(x)
   for (int i=0; i<2; i++)
        values[i] = -2.0 * x[i];
   
    // element at 1,2: grad_{x1} g_{1}(x)
   // values[1] = -1.0;
  }

  return true;
}

bool MyNLP::eval_h(Index n, const Number* x, bool new_x,
                   Number obj_factor, Index m, const Number* lambda,
                   bool new_lambda, Index nele_hess, Index* iRow,
                   Index* jCol, Number* values)
{
  if (values == NULL) {
    
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    // element at 1,1: grad^2_{x1,x1} L(x,lambda)
    iRow[0] = 1;
    jCol[0] = 1;

    // element at 2,2: grad^2_{x2,x2} L(x,lambda)
    iRow[1] = 2;
    jCol[1] = 2;

    // Note: off-diagonal elements are zero for this problem
  }
  else {
    // return the values

    // element at 1,1: grad^2_{x1,x1} L(x,lambda)
    values[0] = -2.0 * lambda[0];

    // element at 2,2: grad^2_{x2,x2} L(x,lambda)
    values[1] = -2.0 * obj_factor;

    // Note: off-diagonal elements are zero for this problem
  }

  return true;
}

void MyNLP::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			      const IpoptData* ip_data,
			      IpoptCalculatedQuantities* ip_cq)
{
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution. Since the solution is displayed to the console,
  // we currently do nothing here.
  for (Index i=0; i<15; i++) {
    cout << "x[" << i << "]\t" << x[i] << endl; 
  }
}