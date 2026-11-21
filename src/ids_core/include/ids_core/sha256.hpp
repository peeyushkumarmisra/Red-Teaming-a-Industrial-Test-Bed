// =============================================================================
//  sha256.hpp - SHA-256 hash for calibration fingerprints
// -----------------------------------------------------------------------------
//  A small self-contained SHA-256, so the fingerprint needs no crypto library.
// =============================================================================

#pragma once

#include <string>

namespace ids_core {

std::string sha256Hex(const std::string& data);

}  // namespace ids_core
