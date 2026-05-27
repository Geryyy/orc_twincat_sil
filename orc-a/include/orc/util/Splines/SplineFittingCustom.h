/*
    Project: ORC - Open Robot Control Library
    Author: anonymous
    Date: Thu Nov 23 15:14:14 2023 +0100
    License: See accompanying LICENSE file
    */

#ifndef EIGEN_SPLINE_FITTING_CUSTOM_H
#define EIGEN_SPLINE_FITTING_CUSTOM_H

#include <algorithm>
#include <functional>
#include <numeric>
#include <vector>
#include "orc/util/Logger.h"

#include "SplineFwd.h"

#include "Eigen/LU"
#include "Eigen/QR"

namespace Eigen {

template <typename SplineType>
template <typename PointArrayType, typename IndexArray>
SplineType SplineFitting<SplineType>::InterpolateWithDerivatives2(
    const PointArrayType& points, const PointArrayType& firstDerivatives,
    const IndexArray& firstDerivativeIndices, const PointArrayType& secondDerivatives,
    const IndexArray& secondDerivativeIndices, const unsigned int degree,
    const ParameterVectorType& parameters) {
    typedef typename SplineType::KnotVectorType::Scalar Scalar;
    typedef typename SplineType::ControlPointVectorType ControlPointVectorType;

    typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;

    const DenseIndex n = points.cols() + firstDerivatives.cols() + secondDerivatives.cols();

    KnotVectorType knots;
    KnotVectorType knots_tmp;

    // second deriv. first!
    KnotAveragingWithDerivatives2(parameters, degree, firstDerivativeIndices,
                                  secondDerivativeIndices, knots);
    // ParameterVectorType param_tmp(knots_tmp);
    // KnotAveragingWithDerivatives(parameters, degree, firstDerivativeIndices, knots);

    // KnotVectorType knots_tmp;
    KnotAveragingWithEndDerivatives(parameters, degree, 2, 2, knots_tmp);
    // std::cout << "knots tmp: " << knots_tmp << std::endl;
    // std::cout << "----------------------------------------" << std::endl;
    // std::cout << "\nknot vector:" << knots << std::endl;

    // fill matrix
    MatrixType A = MatrixType::Zero(n, n);

    // Use these dimensions for quicker populating, then transpose for solving.
    MatrixType b(points.rows(), n);

    DenseIndex startRow = 1;
    DenseIndex derivativeStart = 0;

    // First derivatives at begin.
    if (firstDerivativeIndices[0] == 0) {
        A.template block<1, 2>(1, 0) << (-1, 1);

        Scalar y = (knots(degree + 1) - knots(0)) / degree;
        b.col(1) = y * firstDerivatives.col(0);

        startRow++;
        derivativeStart++;
    }

    // Second derivatives at begin.
    if (secondDerivativeIndices[0] == 0) {
        Scalar b2 = degree * (degree - 1) / (knots(degree + 1) - knots(2));
        Scalar a22 = 1 / (knots(degree + 2) - knots(2));
        Scalar a21 = -1 * (1 / (knots(degree + 2) - knots(2)) + 1 / (knots(degree + 1) - knots(1)));
        Scalar a20 = 1 / (knots(degree + 1) - knots(1));
        A.template block<1, 3>(2, 0) << a20, a21, a22;

        Scalar y = 1 / b2;
        b.col(1) = y * secondDerivatives.col(0);

        startRow++;
    }

    // First derivatives at end.
    if (firstDerivativeIndices[firstDerivatives.cols() - 1] == points.cols() - 1) {
        A.template block<1, 2>(n - 2, n - 2) << (-1, 1);

        Scalar y = (knots(knots.size() - 1) - knots(knots.size() - (degree + 2))) / degree;
        b.col(b.cols() - 2) = y * firstDerivatives.col(firstDerivatives.cols() - 1);
    }

    // Second derivatives at end.
    if (secondDerivativeIndices[secondDerivatives.cols() - 1] == points.cols() - 1) {
        // TODO: check if correct (inverted order of sec deriv at beginning)
        Scalar b2 = degree * (degree - 1) / (knots(degree + 1) - knots(2));
        Scalar a22 = 1 / (knots(degree + 2) - knots(2));
        Scalar a21 = -1 * (1 / (knots(degree + 2) - knots(2)) + 1 / (knots(degree + 1) - knots(1)));
        Scalar a20 = 1 / (knots(degree + 1) - knots(1));
        A.template block<1, 3>(n - 3, n - 3) << a22, a21, a20;

        // Scalar y = (knots(knots.size() - 1) - knots(knots.size() - (degree + 2))) / degree;
        Scalar y = 1 / b2;
        b.col(b.cols() - 3) = y * secondDerivatives.col(secondDerivatives.cols() - 1);
    }

    /* First derivatives in between */
    DenseIndex row = startRow;
    DenseIndex derivativeIndex = derivativeStart;
    for (DenseIndex i = 1; i < parameters.size() - 1; ++i) {
        const DenseIndex span = SplineType::Span(parameters[i], degree, knots);

        if (derivativeIndex < firstDerivativeIndices.size() &&
            firstDerivativeIndices[derivativeIndex] == i) {
            A.template block(row, span - degree, 2, degree + 1) =
                SplineType::BasisFunctionDerivatives(parameters[i], 1, degree, knots);

            b.col(row++) = points.col(i);
            b.col(row++) = firstDerivatives.col(derivativeIndex++);
        } else {
            A.row(row).segment(span - degree, degree + 1) =
                SplineType::BasisFunctions(parameters[i], degree, knots);
            b.col(row++) = points.col(i);
        }
    }
    b.col(0) = points.col(0);
    b.col(b.cols() - 1) = points.col(points.cols() - 1);
    A(0, 0) = 1;
    A(n - 1, n - 1) = 1;

    // Solve
    // reference: https://eigen.tuxfamily.org/dox-devel/group__TutorialLinearAlgebra.html
    // PartialPivLU<MatrixType> lu(A); // 16-18x cycle time exceeded
    FullPivLU<MatrixType> lu(A);  // 10-14x cycle time exceeded
    // HouseholderQR<MatrixType> lu(A); // 5,13,10x cycle time exceeded
    // ColPivHouseholderQR<MatrixType> lu(A); // 15,14,5x cycle time exceeded
    // FullPivHouseholderQR<MatrixType> lu(A); // 22,15,7x cycle time exceeded
    // CompleteOrthogonalDecomposition<MatrixType> lu(A); // 14,8,10x cycle time exceeded
    // LLT<MatrixType> lu(A); // 6,4,14x cycle time exceeded
    // BDCSVD<MatrixType> lu(A); // 8,12,24x cycle time exceeded
    ControlPointVectorType controlPoints = lu.solve(MatrixType(b.transpose())).transpose();

    SplineType spline(knots, controlPoints);

    return spline;
}

/**
 * \brief Computes knot averages when derivative constraints are present.
 * Note that this is a technical interpretation of the referenced article
 * since the algorithm contained therein is incorrect as written.
 * \ingroup Splines_Module
 *
 * \param[in] parameters The parameters at which the interpolation B-Spline
 *            will intersect the given interpolation points. The parameters
 *            are assumed to be a non-decreasing sequence.
 * \param[in] degree The degree of the interpolating B-Spline. This must be
 *            greater than zero.
 * \param[in] firstDerivativeIndices The indices corresponding to parameters at
 *            which there are derivative constraints. The indices are assumed
 *            to be a non-decreasing sequence.
 * \param[out] knots The calculated knot vector. These will be returned as a
 *             non-decreasing sequence
 *
 * \sa Les A. Piegl, Khairan Rajab, Volha Smarodzinana. 2008.
 * Curve interpolation with directional constraints for engineering design.
 * Engineering with Computers
 **/
template <typename KnotVectorType, typename ParameterVectorType, typename IndexArray>
void KnotAveragingWithDerivatives2(const ParameterVectorType& parameters, const unsigned int degree,
                                   const IndexArray& firstDerivativeIndices,
                                   const IndexArray& secondDerivativeIndices,
                                   KnotVectorType& knots) {
    typedef typename ParameterVectorType::Scalar Scalar;

    DenseIndex numParameters = parameters.size();
    DenseIndex numFirstDerivatives = firstDerivativeIndices.size();
    DenseIndex numSecondDerivatives = secondDerivativeIndices.size();

    if (numFirstDerivatives < 1 && numSecondDerivatives < 1) {
        KnotAveraging(parameters, degree, knots);
        return;
    }

    if (numSecondDerivatives < 1) {
        KnotAveragingWithDerivatives(parameters, degree, firstDerivativeIndices, knots);
        return;
    }

    DenseIndex startIndex;
    DenseIndex endIndex;

    DenseIndex numInternalDerivatives = numFirstDerivatives;

    if (firstDerivativeIndices[0] == 0) {
        startIndex = 0;
        --numInternalDerivatives;
    } else {
        startIndex = 1;
    }
    if (firstDerivativeIndices[numFirstDerivatives - 1] == numParameters - 1) {
        endIndex = numParameters - degree;
        --numInternalDerivatives;
    } else {
        endIndex = numParameters - degree - 1;
    }

    // There are (endIndex - startIndex + 1) knots obtained from the averaging
    // and 2 for the first and last parameters.
    DenseIndex numAverageKnots = endIndex - startIndex + 3;
    KnotVectorType averageKnots(numAverageKnots);
    averageKnots[0] = parameters[0];

    int newKnotIndex = 0;
    for (DenseIndex i = startIndex; i <= endIndex; ++i)
        averageKnots[++newKnotIndex] = parameters.segment(i, degree).mean();
    averageKnots[++newKnotIndex] = parameters[numParameters - 1];

    newKnotIndex = -1;

    ParameterVectorType temporaryParameters(numParameters + 1);
    KnotVectorType derivativeKnots(numInternalDerivatives);
    for (DenseIndex i = 0; i < numAverageKnots - 1; ++i) {
        temporaryParameters[0] = averageKnots[i];
        ParameterVectorType parameterIndices(numParameters);
        int temporaryParameterIndex = 1;
        for (DenseIndex j = 0; j < numParameters; ++j) {
            Scalar parameter = parameters[j];
            if (parameter >= averageKnots[i] && parameter < averageKnots[i + 1]) {
                parameterIndices[temporaryParameterIndex] = j;
                temporaryParameters[temporaryParameterIndex++] = parameter;
            }
        }
        temporaryParameters[temporaryParameterIndex] = averageKnots[i + 1];

        for (int j = 0; j <= temporaryParameterIndex - 2; ++j) {
            for (DenseIndex k = 0; k < firstDerivativeIndices.size(); ++k) {
                if (parameterIndices[j + 1] == firstDerivativeIndices[k] &&
                    parameterIndices[j + 1] != 0 && parameterIndices[j + 1] != numParameters - 1) {
                    derivativeKnots[++newKnotIndex] = temporaryParameters.segment(j, 3).mean();
                    break;
                }
            }
        }
    }

    KnotVectorType temporaryKnots(averageKnots.size() + derivativeKnots.size());

    // std::merge(averageKnots.data(), averageKnots.data() + averageKnots.size(),
    //           derivativeKnots.data(), derivativeKnots.data() + derivativeKnots.size(),
    //           temporaryKnots.data());

    custom_merge(averageKnots.data(), averageKnots.data() + averageKnots.size(),
                 derivativeKnots.data(), derivativeKnots.data() + derivativeKnots.size(),
                 temporaryKnots.data());

    /* add knots for 2nd derivatives at begin and end */
    KnotVectorType secDerivativeKnots(numSecondDerivatives);
    int secderiv_ind = 0;
    if (secondDerivativeIndices[0] == 0) {
        if (temporaryKnots.size() > 2) {
            secDerivativeKnots[secderiv_ind++] =
                (temporaryKnots[0] + temporaryKnots[1] + temporaryKnots[2]) / 3;
        } else {
            secDerivativeKnots[secderiv_ind++] = 0;
            // std::cout << temporaryKnots << std::endl;
        }
    }

    int numTmpKnots = temporaryKnots.size();
    if (secondDerivativeIndices[numSecondDerivatives - 1] == (numParameters - 1)) {
        if (temporaryKnots.size() > 2) {
            secDerivativeKnots[secderiv_ind++] =
                (temporaryKnots[numTmpKnots - 1] + temporaryKnots[numTmpKnots - 2] +
                 temporaryKnots[numTmpKnots - 3]) /
                3;
        } else {
            secDerivativeKnots[secderiv_ind++] = 1;
            // std::cout << temporaryKnots << std::endl;
        }
    }

    KnotVectorType temporaryKnots2(temporaryKnots.size() + numSecondDerivatives);

    custom_merge(temporaryKnots.data(), temporaryKnots.data() + temporaryKnots.size(),
                 secDerivativeKnots.data(), secDerivativeKnots.data() + secDerivativeKnots.size(),
                 temporaryKnots2.data());

    temporaryKnots = temporaryKnots2;

    // Number of knots (one for each point and derivative) plus spline order.
    DenseIndex numKnots = numParameters + numFirstDerivatives + numSecondDerivatives + degree + 1;
    knots.resize(numKnots);

    knots.head(degree).fill(temporaryKnots[0]);
    knots.tail(degree).fill(temporaryKnots.template tail<1>()[0]);
    knots.segment(degree, temporaryKnots.size()) = temporaryKnots;
}

template <typename KnotVectorType, typename ParameterVectorType>
void KnotAveragingWithEndDerivatives(const ParameterVectorType& parameters,
                                     const unsigned int degree,
                                     const unsigned int start_deriv_order,
                                     const unsigned int end_deriv_order, KnotVectorType& knots) {
    typedef typename ParameterVectorType::Scalar Scalar;

    DenseIndex numParameters = parameters.size();
    DenseIndex p = degree;
    DenseIndex m = numParameters - 1;  // highest parameter index
    DenseIndex k = start_deriv_order;
    DenseIndex l = end_deriv_order;
    DenseIndex n = (numParameters + k + l) - 1;  // highest control point index
    DenseIndex numKnots = numParameters + k + l + p + 1;
    knots.resize(numKnots);

    knots.head(p + 1).fill(parameters(0));
    knots.tail(p + 1).fill(parameters(last));

    DenseIndex nc = n - k - l;
    Scalar inc = (m + 1.0) / (nc + 1.0);
    DenseIndex low = 0;
    DenseIndex high = 0;
    Scalar d = -1;
    VectorXd w(nc + 1);

    for (int i = 0; i <= nc; i++) {
        d += inc;
        high = static_cast<DenseIndex>(d + 0.5);
        Scalar sum = 0;
        for (int j = low; j <= high; j++) {
            sum += parameters(j);
        }

        w(i) = sum / (high - low + 1);
        low = high + 1;
    }

    DenseIndex is = 1 - k;
    DenseIndex ie = nc - p + l;
    DenseIndex r = p;

    for (int i = is; i <= ie; i++) {
        DenseIndex js = std::max(0, i);
        DenseIndex je = std::min(nc, i + p - 1);
        r++;

        Scalar sum = 0;
        for (int j = js; j <= je; j++) {
            sum += w(j);
        }
        knots(r) = sum / (je - js + 1);
    }
}

template <typename SplineType>
template <typename PointArrayType>
SplineType SplineFitting<SplineType>::InterpolateWithEndDerivatives(
    const PointArrayType& points, const PointArrayType& startDerivatives,
    const PointArrayType& endDerivatives, const unsigned int degree,
    const ParameterVectorType& parameters) {
    orc::log::write_debug("InterpolateWithEndDerivatives(): gesamt");
    int64_t start_time3;
    orc::log::tic(&start_time3);

    typedef typename SplineType::KnotVectorType::Scalar Scalar;
    typedef typename SplineType::ControlPointVectorType ControlPointVectorType;

    typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
    const DenseIndex p = degree;
    const DenseIndex k = startDerivatives.cols();
    const DenseIndex l = endDerivatives.cols();
    const DenseIndex N =
        points.cols() + startDerivatives.cols() + endDerivatives.cols();  // nr of control pointss
    const DenseIndex n = N - 1;
    const DenseIndex M = parameters.size();
    const DenseIndex m = M - 1;

    KnotVectorType knots;
    KnotAveragingWithEndDerivatives(parameters, degree, k, l, knots);

    MatrixType P = MatrixType::Zero(points.rows(), N);

    /* points at start defined by derivatives at start */
    P.col(0) = points.col(0);

    Scalar t0 = parameters(0);
    for (DenseIndex i = 1; i <= k; i++) {
        auto bfun_deriv = SplineType::BasisFunctionDerivatives(t0, k, p, knots);

        PointArrayType sum = PointArrayType::Zero(points.rows(), 1);
        for (DenseIndex h = 0; h <= i - 1; h++) {
            sum = sum + bfun_deriv(i, h) * P.col(h);
        }

        P.col(i) = 1 / (bfun_deriv(i, i)) * (startDerivatives.col(i - 1) - sum);
    }

    /* points at end defined by derivatives at end */
    P.col(n) = points.col(m);

    Scalar tm = parameters(m);
    for (DenseIndex i = 1; i <= l; i++) {
        auto bfun_deriv = SplineType::BasisFunctionDerivatives(tm, l, p, knots);
        DenseIndex N_last_ind = (bfun_deriv.cols() - 1);

        PointArrayType sum = PointArrayType::Zero(points.rows(), 1);
        for (DenseIndex h = 0; h <= i - 1; h++) {
            sum = sum + bfun_deriv(i, N_last_ind - h) * P.col(n - h);
        }

        P.col(n - i) = 1 / (bfun_deriv(i, N_last_ind - i)) * (endDerivatives.col(i - 1) - sum);
    }

    MatrixType N_full = MatrixType::Zero(M, N);

    // evaluate all basis functions
    for (int r = 0; r <= m; r++) {
        Scalar t_r = parameters[r];
        const DenseIndex span = SplineType::Span(t_r, p, knots);
        N_full.row(r).segment(span - p, p + 1) = SplineType::BasisFunctions(t_r, p, knots);
    }

    MatrixType R_vec(m - 1, points.rows());

    for (int r = 1; r <= m - 1; r++) {
        PointArrayType sum1 = PointArrayType::Zero(points.rows(), 1);

        for (int i = 0; i <= k; i++) {
            sum1 += N_full(r, i) * P.col(i);
        }

        PointArrayType sum2 = PointArrayType::Zero(points.rows(), 1);

        for (DenseIndex j = 0; j <= k; j++) {
            sum2 += N_full(r, n - j) * P.col(n - j);
        }

        R_vec.row(r - 1) = points.col(r) - sum1 - sum2;
    }

    DenseIndex size_rows = m - 1;
    DenseIndex size_cols = n - l - k - 1;
    MatrixType N_mat = N_full.block(1, k + 1, size_rows, size_cols);

    MatrixType R_mat = MatrixType::Zero(size_cols, points.rows());
    R_mat = N_mat.transpose() * R_vec;

    MatrixType NTN = MatrixType::Zero(size_cols, size_cols);
    NTN = N_mat.transpose() * N_mat;

    MatrixType P_mat = MatrixType::Zero(size_cols, points.rows());

    orc::log::write_debug("InterpolateWithEndDerivatives(): Calculate spline control points.");

    FullPivLU<MatrixType> lu(NTN);
    ControlPointVectorType controlPoints_tmp = lu.solve(MatrixType(R_mat)).transpose();

    ControlPointVectorType controlPoints(P);  // <--- error
    // controlPoints = P;//P.transpose();
    for (int i = 0; i < controlPoints_tmp.cols(); i++) {
        controlPoints.col(i + k + 1) = controlPoints_tmp.col(i);
    }

    SplineType spline(knots, controlPoints);

    return spline;
}

}  // namespace Eigen
#endif  // EIGEN_SPLINE_FITTING_CUSTOM_H
