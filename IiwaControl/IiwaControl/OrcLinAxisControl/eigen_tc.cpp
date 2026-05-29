/*
    Project: ORC - Open Robot Control Library
    Author: anonymous <anonymous@anonymous.com>
    Date: Tue Jun 28 10:44:59 2022 +0200
    License: See accompanying LICENSE file
    */

    //#include "TcDef.h"
//#include "TcBase.h"
//#include "TcError.h"
//#include "OsBase.h"
#include <cmath>
#include <string>
#include "orc/util/eigen_tc_settings.h"
#include "orc/util/import_eigen.h"

/**
 * @brief dummy implementation - call to _alloca leads to exception!
 * necessary for eigen lib to compile. EIGEN_STACK_ALLOCATION_LIMIT set to 0!
 *
 * @param size
 * @return void*
 */
void *_alloca(size_t size)
{
	return nullptr;
}

/**
 * @brief Custom implementation to use eigen in twincat environment.
 *
 * @param x
 * @return float
 */
float rintf(const float &x)
{
	return static_cast<float>(round(x));
}

/**
 * @brief Custom implementation to use eigen in twincat environment.
 *
 * @param x
 * @return int
 */
int rint(const float &x)
{
	return static_cast<int>(round(x));
}

/**
 * @brief Custom implementation to use eigen in twincat environment.
 *
 * @param x
 * @return long double
 */
long double fabsl(const long double &x)
{
	if (x > 0)
	{
		return x;
	}
	else
	{
		return -1.0 * x;
	}
}

/**
 * @brief Custom implementation to use eigen in twincat environment.
 *
 * @param f1
 * @param f2
 * @return float
 */
float fmaxf(const float &f1, const float &f2)
{
	if (f1 > f2)
	{
		return f1;
	}
	else
	{
		return f2;
	}
}

/**
 * @brief Custom implementation to use eigen in twincat environment.
 *
 * @param f1
 * @param f2
 * @return float
 */
float fminf(const float &f1, const float &f2)
{
	if (f1 > f2)
	{
		return f2;
	}
	else
	{
		return f1;
	}
}

/**
 * @brief Convert printf like argument list to string.
 *
 * @param format
 * @param ...
 * @return std::string
 */
static std::string stringf(const char *format, ...)
{
	va_list args;
	va_start(args, format);

	int size = 10;
	std::unique_ptr<char[]> buf(new char[size]);
	vsprintf(buf.get(), format, args);
	va_end(args);
	return std::string(buf.get());
}

/**
 * @brief TODO: delete this
 *
 * @param vect
 * @return std::string
 */
static std::string EigenMatrixtoString(const Eigen::Ref<Eigen::Matrix<double, -1, -1>> &vect)
{
	return stringf("[%f]", vect(0, 0));
}
