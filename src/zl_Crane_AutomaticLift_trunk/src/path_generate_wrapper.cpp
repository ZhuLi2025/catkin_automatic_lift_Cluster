#include "path_generate_wrapper.hpp"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "path_generate_api.h"

namespace zl_Crane_AutomaticLift_trunk {

namespace {

bool read_file_text(const std::string& path, std::string& out, std::string* err) {
  std::ifstream file(path);
  if (!file.is_open()) {
    if (err) {
      *err = "Failed to open JSON file: " + path;
    }
    return false;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  out = buffer.str();
  if (out.empty()) {
    if (err) {
      *err = "JSON file is empty: " + path;
    }
    return false;
  }
  return true;
}

std::string dirname_of_path(const std::string& path) {
  const std::size_t pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return ".";
  }
  if (pos == 0) {
    return path.substr(0, 1);
  }
  return path.substr(0, pos);
}

std::string join_path(const std::string& dir, const std::string& name) {
  if (dir.empty() || dir == ".") {
    return name;
  }
  const char last = dir.back();
  if (last == '/' || last == '\\') {
    return dir + name;
  }
  return dir + "/" + name;
}

std::string timestamp_second() {
  std::time_t now = std::time(nullptr);
  std::tm tm_time{};
#if defined(_WIN32)
  localtime_s(&tm_time, &now);
#else
  localtime_r(&now, &tm_time);
#endif
  std::ostringstream ss;
  ss << std::setfill('0')
     << std::setw(4) << (tm_time.tm_year + 1900)
     << std::setw(2) << (tm_time.tm_mon + 1)
     << std::setw(2) << tm_time.tm_mday
     << "_"
     << std::setw(2) << tm_time.tm_hour
     << std::setw(2) << tm_time.tm_min
     << std::setw(2) << tm_time.tm_sec;
  return ss.str();
}

std::string escape_json_string(const std::string& input) {
  std::ostringstream out;
  for (char c : input) {
    switch (c) {
      case '\\': out << "\\\\"; break;
      case '\"': out << "\\\""; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out << "\\u"
              << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c))
              << std::dec << std::setfill('0');
        } else {
          out << c;
        }
        break;
    }
  }
  return out.str();
}

std::string build_failure_json(const std::string& reason) {
  std::ostringstream ss;
  ss << "{\n"
     << "  \"status\": \"failure\",\n"
     << "  \"message\": \"轨迹无法规划\",\n"
     << "  \"reason\": \"" << escape_json_string(reason) << "\"\n"
     << "}\n";
  return ss.str();
}

bool write_file_text(const std::string& path, const std::string& text, std::string* err) {
  std::ofstream file(path);
  if (!file.is_open()) {
    if (err) {
      *err = "Failed to open output JSON file: " + path;
    }
    return false;
  }
  file << text;
  if (!file.good()) {
    if (err) {
      *err = "Failed to write output JSON file: " + path;
    }
    return false;
  }
  return true;
}

bool plan_from_json_file(const std::string& json_path, std::string& out_json, std::string& err) {
  std::string input_json;
  if (!read_file_text(json_path, input_json, &err)) {
    return false;
  }
  const char* output = pg_plan_from_json(input_json.c_str());
  if (!output) {
    err = "pg_plan_from_json returned null.";
    return false;
  }
  out_json.assign(output);
  pg_free_string(output);
  return true;
}

}  // namespace

std::string PathGenerateWrapper::PlanFromJsonFileAutoOutput(const std::string& input_json_path) {
  const std::string dir = dirname_of_path(input_json_path);
  const std::string filename = "output_" + timestamp_second() + ".json";
  const std::string output_json_path = join_path(dir, filename);

  std::string out_json;
  std::string local_err;
  bool ok = plan_from_json_file(input_json_path, out_json, local_err);
  if (!ok) {
    if (local_err.empty()) {
      local_err = "Unknown error.";
    }
    out_json = build_failure_json(local_err);
  }
  if (!write_file_text(output_json_path, out_json, nullptr)) {
    return std::string();
  }
  return output_json_path;
}

}  // namespace zl_Crane_AutomaticLift_trunk

#if 0
// Example usage (unit-test style):
// int main() {
//   std::string output_path = zl_Crane_AutomaticLift_trunk::PathGenerateWrapper::PlanFromJsonFileAutoOutput(
//       "/path/to/input.json");
//   if (output_path.empty()) {
//     std::cerr << "Plan failed: output write error." << std::endl;
//     return 1;
//   }
//   std::cout << "Output written: " << output_path << std::endl;
//   return 0;
// }
#endif
