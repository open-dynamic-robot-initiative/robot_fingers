/**
 * \file
 * \copyright Copyright (c) 2020, New York University & Max Planck Gesellschaft.
 */
#pragma once

namespace blmc_robots
{
/**
 * @brief Clamp an arbitrary Eigen vector.
 *
 * @param vector  The vector that is to be clamped.
 * @param lower_limit  Lower limit.
 * @param upper_limit  Upper limit.
 *
 * @return  Copy of vector where values below or above the limits are set to the
 *     corresponding limit values.
 */
template <typename Vector>
Vector clamp(const Vector &vector,
             const double lower_limit,
             const double upper_limit)
{
    return vector.cwiseMin(upper_limit).cwiseMax(lower_limit);
}

}  // namespace blmc_robots
