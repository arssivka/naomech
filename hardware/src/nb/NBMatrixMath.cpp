#include "nb/NBMatrixMath.h"

using namespace NBMath;

/**
   Invert a two by two matrix easily
   given:
   [a b
   c d]
   return:
   (1/(ad - bc)) [ d -b
   -c  a]
   @param m the 2 by 2 matrix to invert.
   @return the inversion of m
*/
const boost::numeric::ublas::matrix<double>
NBMath::invert2by2(boost::numeric::ublas::matrix<double> m) {
    double det = 1.0 / (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0));
    double tmp = m(0, 0);
    m(0, 0) = m(1, 1);
    m(1, 1) = tmp;
    tmp = -m(0, 1);
    m(0, 1) = -m(1, 0);
    m(1, 0) = tmp;
    return det * m;
}

const boost::numeric::ublas::vector<double>
NBMath::solve(boost::numeric::ublas::matrix<double> A,
              const boost::numeric::ublas::vector<double>& b) {
    boost::numeric::ublas::permutation_matrix<double> P(A.size1());
    int singularRow = lu_factorize(A, P);
    if (singularRow != 0) {
        // TODO: This case needs to be dealt with
        throw "the system had no solution";
    }
    boost::numeric::ublas::vector<double> result(A.size2());
    result.assign(b);
    lu_substitute(A, P, result);
    return result;
}

const boost::numeric::ublas::matrix<double>
NBMath::solve(boost::numeric::ublas::matrix<double> A,
              const boost::numeric::ublas::matrix<double>& B) {
    boost::numeric::ublas::permutation_matrix<double> P(A.size1());
    int singularRow = lu_factorize(A, P);
    if (singularRow != 0) {
        // TODO: This case needs to be dealt with
        throw "the system had no solution";
    }
    boost::numeric::ublas::matrix<double> result(A.size2(), B.size1());
    result.assign(B);
    lu_substitute(A, P, result);
    return result;
}

// Solve the linear system Ax=b for the vector x.
// NOTE: This method is hard coded to work for 3x3 matrices and 3-vectors.
//       We can get superior performance this way.

const ufvector3 NBMath::solve(ufmatrix3& A,
                              const ufvector3& b) {
    boost::numeric::ublas::permutation_matrix
            <double, boost::numeric::ublas::bounded_array<double, 9> >
            P(A.size1());
    int singularRow = lu_factorize(A, P);
    if (singularRow != 0) {
        // TODO: This case needs to be dealt with
        throw "the system had no solution";
    }
    ufvector3 result(A.size2());
    result.assign(b);
    lu_substitute(A, P, result);
    return result;
}
