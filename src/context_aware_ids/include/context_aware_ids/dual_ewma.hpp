#ifndef CONTEXT_AWARE_IDS_DUAL_EWMA_HPP_
#define CONTEXT_AWARE_IDS_DUAL_EWMA_HPP_

#include <cmath>
#include <mutex>
#include <vector>

namespace context_aware_ids
{
class DualEWMA
{
public:
    // Initializing fast/slow learning rates
    DualEWMA(double lambda_fast, double lambda_slow, int burn_in);
    bool update(double residual);
    // For logging and telemetry
    double getCurrentDelta() const;
    double getThreshold() const;
    bool isArmed() const;
    double getFast() const;
    double getSlow() const;
private:
    double lambda_f_;   // Fast learning rate
    double lambda_s_;   // Slow learning rate
    double Sf_;         // Fast EWMA state
    double Ss_;         // Slow EWMA state
    double Th_;         // Dynamic threshold during burn-in
    double curr_delta_; // |Sf - Ss|
    int burn_in_counter_;
    int burn_in_tot_;
    std::vector<double> burn_in_buffer_;    // Stores residuals to calculate mean/variance
    bool armed_;                            // True after burn-in is complete
    mutable std::mutex ewma_mutex_;         // Thread safety for ROS 2 asynchronous access
};
} // namespace context_aware_ids
#endif // CONTEXT_AWARE_IDS_DUAL_EWMA_HPP_