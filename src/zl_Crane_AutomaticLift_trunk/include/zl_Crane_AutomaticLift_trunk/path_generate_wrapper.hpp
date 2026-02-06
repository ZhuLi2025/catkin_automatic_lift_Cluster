#pragma once

#include <string>

namespace zl_Crane_AutomaticLift_trunk {

class PathGenerateWrapper {
public:
  // Reads input JSON from a file path, calls the .so planner, and writes output JSON
  // next to the input file with a timestamped name (second precision).
  // Returns the output_json_path (empty string only if write fails).
  static std::string PlanFromJsonFileAutoOutput(const std::string& input_json_path);
};

}  // namespace zl_Crane_AutomaticLift_trunk
