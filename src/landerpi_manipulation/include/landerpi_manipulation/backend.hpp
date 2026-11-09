#pragma once
#include <string>

namespace landerpi_manipulation {
struct BackendResult {
  bool success{false};
  unsigned short error_code{504};
  std::string message{"NOT_IMPLEMENTED"};
};
// Goal-specific MoveIt/legacy methods are added only after model verification.
// A backend must distinguish planned, executed and physically verified results.
class Backend {
 public:
  virtual ~Backend() = default;
  virtual BackendResult readiness() const = 0;
  virtual BackendResult cancel() = 0;
};
}  // namespace landerpi_manipulation
