/**
Define data types used in this library
Mostly aliasing from Eigen Library
**/

#include <stdio.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#ifndef DATA_TYPE_H
#define DATA_TYPE_H

typedef double decimal_t;

//Pre-allocated std::vector for Eigen using vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

//Eigen::Matrix<type, row, col>
//Eigen 1D float vector of size N
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
//Eigen 1D int vector of size N
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;
//Eigen 1D float vector of dynamic size
using VecDf = Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>;
//MxN Eigen matrix
template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;
//MxN Eigen matrix with M unknown
template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
//Vector of Eigen 1D float vector
template <int N>
using vec_Vecf = vec_E<Vecf<N>>;
//Vector of Eigen 1D int vector
template <int N>
using vec_Veci = vec_E<Veci<N>>;

//Eigen 1D float vector of size 2
typedef Vecf<2> Vec2f;
//Eigen 1D int vector of size 2
typedef Veci<2> Vec2i;
//Eigen 1D float vector of size 3
typedef Vecf<3> Vec3f;
//Eigen 1D int vector of size 3
typedef Veci<3> Vec3i;
//Eigen 1D float vector of size 4
typedef Vecf<4> Vec4f;
//Column vector in float of size 6
typedef Vecf<6> Vec6f;

//Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
//Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
//Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
//Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;

//2x2 Matrix in float
typedef Matf<2, 2> Mat2f;
//3x3 Matrix in float
typedef Matf<3, 3> Mat3f;
//4x4 Matrix in float
typedef Matf<4, 4> Mat4f;
//6x6 Matrix in float
typedef Matf<6, 6> Mat6f;

//Dynamic Nx1 Eigen float vector
typedef Vecf<Eigen::Dynamic> VecDf;
//Mx3 Eigen float matrix
typedef MatDNf<3> MatD3f;
//Dynamic MxN Eigen float matrix
typedef Matf<Eigen::Dynamic, Eigen::Dynamic> MatDf;

//Allias of Eigen::Affine2d
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
//Allias of Eigen::Affine3d
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;

#endif