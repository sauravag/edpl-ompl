#ifndef DARE_
#define DARE_

#include "armadillo"
#include <cassert>


//arma::mat dare(const arma::mat& _A, const arma::mat& _B, const arma::mat& _Q, const arma::mat& _R);

//arma::mat generate_gain_with_dare(const arma::mat _A, const arma::mat _B, const arma::mat _Q, const arma::mat _R);

inline arma::mat dare(const arma::mat& _A, const arma::mat& _B, const arma::mat& _Q, const arma::mat& _R) {
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
    if( (eigval(i).real()*eigval(i).real() +
    eigval(i).imag()*eigval(i).imag()  ) > 1) {
      
      tempZ.submat(span::all, span(c1,c1)) = VR.submat(span::all, span(i,i));
      c1++;
    }
  }

  //ensure that the system is stable
  assert(c1 == n);

  U11 = tempZ.submat(span(0,n-1), span(0,n-1));
  U21 = tempZ.submat(span(n,n_Z-1), span(0,n-1));

  return U21 * inv(U11);
}

inline arma::mat generate_gain_with_dare(const arma::mat _A, const arma::mat _B, const arma::mat _Q, const arma::mat _R) {
using namespace arma;

  int n = _A.n_rows, m = _B.n_cols;
  mat S(n,n);
  mat temp1(m,n);
  mat temp2(m,m);

  S = dare(_A, _B, _Q, _R);
  temp1 = trans(_B) * S;
  temp2 = _R + temp1 * _B;
  //should it be inv instead of trans? are they the same?
  //temp1 = trans(temp2) * temp1; 
  temp1 = inv(temp2) * temp1; 
  return (temp1 * _A);

}

#endif
