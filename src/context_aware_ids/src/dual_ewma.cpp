#include "context_aware_ids/dual_ewma.hpp"

#include <numeric>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace context_aware_ids
{
DualEWMA::DualEWMA(double lambda_fast, double lambda_slow, int burn_in) : lambda_f_(lambda_fast),
    lambda_s_(lambda_slow),
    Sf_(0.0),
    Ss_(0.0),
    Th_(0.0),
    curr_delta_(0.0),
    burn_in_counter_(0),
    burn_in_tot_(burn_in),
    armed_(false)
{
    if (burn_in <= 0) {
        throw std::invalid_argument("[DualEWMA] ERROR: burn_in must be > 0");
    }
    burn_in_buffer_.reserve(burn_in);
}

bool DualEWMA::update(double residual)
{
    std::lock_guard<std::mutex> lock(ewma_mutex_);
    // Burn_in Phase: - Data collection for setting baseline
    if (burn_in_counter_ < burn_in_tot_)
    {
        burn_in_buffer_.push_back(residual);
        burn_in_counter_++;
        // Setting Tripwire
        if (burn_in_counter_ == burn_in_tot_ && !armed_)
        {
            // Calculating Mean
            double sum = std::accumulate(burn_in_buffer_.begin(), burn_in_buffer_.end(), 0.0);
            double mean = sum / burn_in_tot_;
            // Calculating Variance and Standard Deviation
            double sq_sum = std::inner_product(burn_in_buffer_.begin(), burn_in_buffer_.end(), burn_in_buffer_.begin(), 0.0);
            double varr = (sq_sum / burn_in_tot_) - (mean * mean);
            double std_dev = std::sqrt(std::max(0.0, varr));
            // Setting Threshold
            Th_ = mean + 1.0 * std_dev;
            // Initializing both EWMAs to the steady-state mean
            Sf_ = mean;
            Ss_ = mean;
            armed_ = true;
            std::cout << "[DualEWMA] Burn-in complete. Tripwire armed with Threshold: " << Th_ << std::endl;
            // Freeing the buffer memory
            burn_in_buffer_.clear();
            burn_in_buffer_.shrink_to_fit();
        }
        return false; // No Alarm yet so its normal
    }
    Sf_ = lambda_f_ * residual + (1.0 - lambda_f_) * Sf_;   // Updating Fast EWMA
    Ss_ = lambda_s_ * residual + (1.0 - lambda_s_) * Ss_;   // Updating Slow EWMA
    curr_delta_ = std::fabs(Sf_ - Ss_);                     // Computing Absolute Delta
    return curr_delta_ > Th_;                               // Return True if threshold is breached
}

double DualEWMA::getCurrentDelta() const
{
    std::lock_guard<std::mutex> lock(ewma_mutex_);
    return curr_delta_;
}

double DualEWMA::getThreshold() const
{
    std::lock_guard<std::mutex> lock(ewma_mutex_);
    return Th_;
}

bool DualEWMA::isArmed() const
{
    std::lock_guard<std::mutex> lock(ewma_mutex_);
    return armed_;
}


double DualEWMA::getFast() const 
{ 
    std::lock_guard<std::mutex> lock(ewma_mutex_);
    return Sf_; 
}

double DualEWMA::getSlow() const 
{ 
    std::lock_guard<std::mutex> lock(ewma_mutex_);
    return Ss_; 
}
}   // namespace context_aware_ids