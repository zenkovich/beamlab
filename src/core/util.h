// Logging, file-system and string helpers.
#pragma once

#include <cstdarg>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace bl {

void log_info(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void log_warn(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void log_error(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Recent log lines for the in-app console.
struct LogLine {
    int level; // 0 info, 1 warn, 2 error
    std::string text;
};
std::vector<LogLine> log_recent(int max_lines);
void set_log_quiet(bool quiet); // suppress info lines on stdout

bool read_file(const std::string& path, std::vector<uint8_t>& out);
bool read_text_file(const std::string& path, std::string& out);
bool file_exists(const std::string& path);
bool dir_exists(const std::string& path);
std::vector<std::string> list_dir(const std::string& dir, bool files, bool dirs);
std::string path_join(const std::string& a, const std::string& b);
std::string path_dir(const std::string& p);
std::string path_filename(const std::string& p);
std::string path_ext_lower(const std::string& p); // ".truck"
std::string path_stem(const std::string& p);

// Root folder that contains "assets/" (found from executable/cwd/build-time path).
const std::string& app_root();
std::string asset_path(const std::string& rel);

std::string to_lower(std::string s);
std::string trim(std::string_view s);
bool starts_with_ci(std::string_view s, std::string_view prefix);
bool iequals(std::string_view a, std::string_view b);
std::vector<std::string> split_any(std::string_view s, std::string_view seps, bool keep_empty = false);
std::string format(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Case-insensitive file lookup inside a folder (RoR content often has inconsistent casing).
// Returns full path or empty string.
std::string find_file_ci(const std::string& dir, const std::string& name);

double time_seconds();

} // namespace bl
