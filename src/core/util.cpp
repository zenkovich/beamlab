#include "core/util.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <unordered_map>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

namespace fs = std::filesystem;

namespace bl {

namespace {
std::mutex g_log_mutex;
std::deque<LogLine> g_log_lines;
bool g_quiet = false;

void log_v(int level, const char* fmt, va_list ap) {
    char buf[2048];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    std::lock_guard<std::mutex> lk(g_log_mutex);
    static const char* prefix[] = {"", "[warn] ", "[error] "};
    if (!(g_quiet && level == 0)) {
        FILE* out = level == 0 ? stdout : stderr;
        fprintf(out, "%s%s\n", prefix[level], buf);
        fflush(out);
    }
    g_log_lines.push_back({level, buf});
    if (g_log_lines.size() > 2000) g_log_lines.pop_front();
}
} // namespace

void log_info(const char* fmt, ...) { va_list ap; va_start(ap, fmt); log_v(0, fmt, ap); va_end(ap); }
void log_warn(const char* fmt, ...) { va_list ap; va_start(ap, fmt); log_v(1, fmt, ap); va_end(ap); }
void log_error(const char* fmt, ...) { va_list ap; va_start(ap, fmt); log_v(2, fmt, ap); va_end(ap); }
void set_log_quiet(bool q) { g_quiet = q; }

std::vector<LogLine> log_recent(int max_lines) {
    std::lock_guard<std::mutex> lk(g_log_mutex);
    int n = std::min<int>(max_lines, (int)g_log_lines.size());
    return std::vector<LogLine>(g_log_lines.end() - n, g_log_lines.end());
}

std::string format(const char* fmt, ...) {
    char buf[4096];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

// BL_TRACE_FILES=<log>: append every successfully opened asset path (used to package only the files in use).
static void trace_file(const std::string& path) {
    static const char* log = getenv("BL_TRACE_FILES");
    if (!log || !*log) return;
    static std::mutex m;
    std::lock_guard<std::mutex> lock(m);
    if (FILE* f = fopen(log, "a")) {
        fprintf(f, "%s\n", path.c_str());
        fclose(f);
    }
}

bool read_file(const std::string& path, std::vector<uint8_t>& out) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    trace_file(path);
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    out.resize(n > 0 ? (size_t)n : 0);
    size_t rd = n > 0 ? fread(out.data(), 1, (size_t)n, f) : 0;
    fclose(f);
    return rd == out.size();
}

bool read_text_file(const std::string& path, std::string& out) {
    std::vector<uint8_t> d;
    if (!read_file(path, d)) return false;
    out.assign(d.begin(), d.end());
    return true;
}

bool file_exists(const std::string& p) {
    std::error_code ec;
    return fs::is_regular_file(p, ec);
}
bool dir_exists(const std::string& p) {
    std::error_code ec;
    return fs::is_directory(p, ec);
}

std::vector<std::string> list_dir(const std::string& dir, bool files, bool dirs) {
    std::vector<std::string> r;
    std::error_code ec;
    for (auto& e : fs::directory_iterator(dir, ec)) {
        if ((files && e.is_regular_file()) || (dirs && e.is_directory())) r.push_back(e.path().filename().string());
    }
    std::sort(r.begin(), r.end());
    return r;
}

std::string path_join(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}
std::string path_dir(const std::string& p) {
    size_t s = p.find_last_of('/');
    return s == std::string::npos ? std::string() : p.substr(0, s);
}
std::string path_filename(const std::string& p) {
    size_t s = p.find_last_of("/\\");
    return s == std::string::npos ? p : p.substr(s + 1);
}
std::string path_ext_lower(const std::string& p) {
    std::string f = path_filename(p);
    size_t d = f.find_last_of('.');
    return d == std::string::npos ? std::string() : to_lower(f.substr(d));
}
std::string path_stem(const std::string& p) {
    std::string f = path_filename(p);
    size_t d = f.find_last_of('.');
    return d == std::string::npos ? f : f.substr(0, d);
}

static std::string exe_dir() {
#if defined(__APPLE__)
    char buf[4096];
    uint32_t sz = sizeof(buf);
    if (_NSGetExecutablePath(buf, &sz) == 0) {
        std::error_code ec;
        auto p = fs::weakly_canonical(buf, ec);
        return p.parent_path().string();
    }
#endif
    return ".";
}

const std::string& app_root() {
    static std::string root = [] {
        // cwd, the executable's folder, and BeamLab.app/Contents/Resources for the packaged app
        std::vector<fs::path> starts = {fs::current_path(), fs::path(exe_dir()), fs::path(exe_dir()).parent_path() / "Resources"};
        for (auto s : starts) {
            for (int i = 0; i < 6 && !s.empty(); i++) {
                std::error_code ec;
                if (fs::is_directory(s / "assets" / "shaders", ec)) return s.string();
                if (s == s.parent_path()) break;
                s = s.parent_path();
            }
        }
#ifdef BEAMLAB_ROOT
        return std::string(BEAMLAB_ROOT);
#else
        return std::string(".");
#endif
    }();
    return root;
}

std::string asset_path(const std::string& rel) { return path_join(path_join(app_root(), "assets"), rel); }

std::string to_lower(std::string s) {
    for (auto& c : s) c = (char)tolower((unsigned char)c);
    return s;
}

std::string trim(std::string_view s) {
    size_t b = 0, e = s.size();
    while (b < e && (unsigned char)s[b] <= ' ') b++;
    while (e > b && (unsigned char)s[e - 1] <= ' ') e--;
    return std::string(s.substr(b, e - b));
}

bool iequals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++)
        if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) return false;
    return true;
}

bool starts_with_ci(std::string_view s, std::string_view prefix) {
    return s.size() >= prefix.size() && iequals(s.substr(0, prefix.size()), prefix);
}

std::vector<std::string> split_any(std::string_view s, std::string_view seps, bool keep_empty) {
    std::vector<std::string> out;
    size_t start = 0;
    for (size_t i = 0; i <= s.size(); i++) {
        if (i == s.size() || seps.find(s[i]) != std::string_view::npos) {
            if (keep_empty || i > start) out.emplace_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    return out;
}

std::string find_file_ci(const std::string& dir, const std::string& name) {
    if (name.empty()) return {};
    std::string direct = path_join(dir, name);
    if (file_exists(direct)) return direct;
    static std::mutex m;
    static std::unordered_map<std::string, std::unordered_map<std::string, std::string>> cache;
    std::lock_guard<std::mutex> lk(m);
    auto it = cache.find(dir);
    if (it == cache.end()) {
        std::unordered_map<std::string, std::string> entries;
        std::error_code ec;
        for (auto& e : fs::recursive_directory_iterator(dir, ec)) {
            if (!e.is_regular_file()) continue;
            std::string fn = to_lower(e.path().filename().string());
            if (!entries.count(fn)) entries[fn] = e.path().string();
        }
        it = cache.emplace(dir, std::move(entries)).first;
    }
    auto f = it->second.find(to_lower(path_filename(name)));
    return f == it->second.end() ? std::string() : f->second;
}

double time_seconds() {
    using namespace std::chrono;
    static auto t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}

} // namespace bl
