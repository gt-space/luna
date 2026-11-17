/**
 * @file eigen.c
 * @brief Double-precision symmetric covariance solver using CControl's CLAPACK.
 */
#include "ccontrol.h"
#include "CLapack/clapack.h"

/**
 * @brief Return eigenvalues and column eigenvectors for a symmetric covariance.
 * @param a Symmetric row-major input matrix, up to 21 by 21.
 * @param dr Real eigenvalues.
 * @param di Imaginary eigenvalues, set to zero.
 * @param wr Row-major matrix containing the real column eigenvectors.
 * @param wi Scratch storage, cleared on success.
 * @param size Matrix dimension.
 * @return true when the decomposition succeeds.
 */
bool eig(const double a[], double dr[], double di[], double wr[], double wi[],
         size_t size)
{
    if (size == 0U || size > 21U) {
        return false;
    }

    integer n = (integer)size;
    integer info, lwork = -1, liwork = -1, iwork_query;
    doublereal work_query;

    memcpy(wi, a, size * size * sizeof *a);
    dsyevd_("V", "U", &n, wi, &n, dr, &work_query, &lwork,
            &iwork_query, &liwork, &info);
    if (info != 0) {
        return false;
    }

    lwork = (integer)work_query;
    liwork = iwork_query;
    doublereal *work = malloc((size_t)lwork * sizeof *work);
    integer *iwork = malloc((size_t)liwork * sizeof *iwork);
    if (work == NULL || iwork == NULL) {
        free(iwork);
        free(work);
        return false;
    }

    dsyevd_("V", "U", &n, wi, &n, dr, work, &lwork, iwork, &liwork, &info);
    free(iwork);
    free(work);
    if (info != 0) {
        return false;
    }

    for (size_t row = 0; row < size; ++row) {
        for (size_t col = 0; col < size; ++col) {
            wr[row * size + col] = wi[col * size + row];
        }
    }
    memset(di, 0, size * sizeof *di);
    memset(wi, 0, size * size * sizeof *wi);
    return true;
}
