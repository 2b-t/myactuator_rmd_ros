/**
 * \file low_pass_filter.hpp
 * \mainpage
 *    A simple low-pass filter with a smoothing constant
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD_HARDWARE__LOW_PASS_FILTER
#define MYACTUATOR_RMD_HARDWARE__LOW_PASS_FILTER
#pragma once

#include <cmath>


namespace myactuator_rmd_hardware {

  /**\class LowPassFilter
   * \brief
   *    A simple low-pass filter with a smoothing constant
   *    https://en.wikipedia.org/wiki/Low-pass_filter
  */
  class LowPassFilter {
    public:
      /**\fn LowPassFilter
       * \brief
       *    Class constructor
       *    Computes alpha from sampling rate and cut-off frequency
       * 
       * \param[in] cutoff_frequency
       *    The cut-off frequency in Hertz [Hz]
       * \param[in] sampling_frequency
       *    The sampling frequency in Hertz [Hz]
       * \param[in] previous_filtered_value
       *    The previous value that should be set
      */
      constexpr LowPassFilter(double const cutoff_frequency, double const sampling_frequency,
                              double const previous_filtered_value = 0.0) noexcept
      : alpha_{2.0*M_PI*cutoff_frequency/(2.0*M_PI*cutoff_frequency + sampling_frequency)},
        previous_filtered_value_{previous_filtered_value} {
        return;
      }

      /**\fn LowPassFilter
       * \brief
       *    Class constructor
       *
       * \param[in] alpha
       *    The filter coefficient (0 < alpha <= 1), for de-activating filter set alpha to 1
       * \param[in] previous_filtered_value
       *    The previous value that should be set
      */
      constexpr LowPassFilter(double const alpha, double const previous_filtered_value = 0.0) noexcept
      : alpha_{alpha}, previous_filtered_value_{previous_filtered_value} {
        // TODO: Check that alpha is between 0 and 1
        return;
      }

      LowPassFilter() = delete;
      LowPassFilter(LowPassFilter const&) = default;
      LowPassFilter& operator = (LowPassFilter const&) = default;
      LowPassFilter(LowPassFilter&&) = default;
      LowPassFilter& operator = (LowPassFilter&&) = default;

      /**\fn setPreviousValue
       * \brief
       *    Set the previous value of the internally saved state
       *
       * \param[in] value
       *    The value to be set as previous value
      */
      constexpr void setPreviousValue(double const value) noexcept {
        previous_filtered_value_ = value;
        return;
      }

      /**\fn apply
       * \brief
       *    Apply the low-pass filter to the current value
       * 
       * \param[in] current_value
       *    The current value to be filtered
       * \return
       *    The filtered value
      */
      constexpr double apply(double const current_value) noexcept {
        double const filtered_value {alpha_*current_value + (1.0 - alpha_)*previous_filtered_value_};
        previous_filtered_value_ = filtered_value;
        return filtered_value;
      }

    protected:
      double alpha_;
      double previous_filtered_value_;
  };

}  // namespace myactuator_rmd_hardware

#endif  // MYACTUATOR_RMD_HARDWARE__LOW_PASS_FILTER
