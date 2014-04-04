/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Texas A&M University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef DARE_
#define DARE_

#include "armadillo"
#include <cassert>


/** \brief Solver for the Differential Algebraic Riccatti Equation. */
inline bool dare(const arma::mat& _A, const arma::mat& _B, const arma::mat& _Q, const arma::mat& _R, arma::mat &S)
{
  using namespace arma;

  int n = _A.n_rows, m = _B.n_cols;
  mat Z11(n,n);
  mat Z12(n,n);
  mat Z21(n,n);
  mat Z22(n,n);
  mat temp1(m,m);
  mat temp2(n,n);
  mat temp3(m,n);

  //pre-compute values to save recomputation
  //"trans" takes Hermitian transpose of matrix,
  //i.e. conjugate of elements is taken during transpose operation
  temp1 = inv(_R);
  temp2 = _B * temp1 * trans(_B);

  //construct submatrices that will constitute Hamiltonian matrix
  Z11 = inv(_A);
  Z21 = _Q * Z11;
  Z12 = Z11 * temp2;
  Z22 = trans(_A) + Z21 * temp2;

  mat Z(2*n, 2*n);

  //construct Hamiltonian matrix in discrete case
  Z.submat( span(0,n-1),    span(0,n-1)   ) = Z11;
  Z.submat( span(n,2*n-1),  span(0,n-1)   ) = Z21;
  Z.submat( span(0,n-1),    span(n,2*n-1) ) = Z12;
  Z.submat( span(n,2*n-1),  span(n,2*n-1) ) = Z22;

  //data structures to store result of generalized eigenvalue computation
  //Col< std::complex<typename T1::pod_type> > eigval(2*n);
  cx_vec eigval(2*n);
  int n_Z = Z.n_rows;
  mat VL(n_Z, n_Z);
  mat VR(n_Z, n_Z);

  //solution to generalized eigenvalue problem
  //internally, calls Fortran's "geev" function
  eig_gen(eigval, VL, VR, Z);

  //cout << "eigen values in DARE: " << eigval << endl;

  mat U11(n,n);
  mat U21(n,n);
  mat tempZ(n_Z, n);

  int c1=0;
  for(int i = 0; i < n_Z; ++i) {

    //get eigenvalues outside unit circle
    if( (eigval(i).real()*eigval(i).real() + eigval(i).imag()*eigval(i).imag()  ) > 1)
    {
      //std::cout<<"VR:  \n"<<VR.submat(span::all, span(i,i))<<std::endl;
      //std::cout<<"Span(i,i) :"<<span(i,i)<<std::endl;
      //std::cout<<"C1 :" <<c1 <<std::endl;
      tempZ.submat(span::all, span(c1,c1)) = VR.submat(span::all, span(i,i));

      c1++;
    }
  }

  //ensure that the system is stable
  if(c1 != n) return false;

  U11 = tempZ.submat(span(0,n-1), span(0,n-1));
  U21 = tempZ.submat(span(n,n_Z-1), span(0,n-1));

  S = U21 * inv(U11);

  return true; // dare was solved successfully
}

/** \brief Generate a gain using the DARE solver */
inline arma::mat generate_gain_with_dare(const arma::mat _A, const arma::mat _B, const arma::mat _Q, const arma::mat _R)
{
  using namespace arma;

  int n = _A.n_rows, m = _B.n_cols;
  mat S(n,n);
  mat temp1(m,n);
  mat temp2(m,m);

  bool flag = dare(_A, _B, _Q, _R, S);
  temp1 = trans(_B) * S;
  temp2 = _R + temp1 * _B;
  temp1 = inv(temp2) * temp1;

  return (temp1 * _A);
}

#endif
