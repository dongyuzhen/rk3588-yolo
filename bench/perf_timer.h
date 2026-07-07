#pragma once

/**
 * @file perf_timer.h
 * @brief 零开销性能计时器 —— 编译开关 BENCH_MODE 控制
 *
 * 用法：
 *   void foo() {
 *       PERF_SCOPE("my_stage");   // 函数出口自动计时
 *       // ... 你的代码 ...
 *   }
 *
 *   或手动：
 *   auto t0 = PerfTimer::start();
 *   do_work();
 *   PerfTimer::stop("my_stage", t0);
 *
 * 编译：
 *   cmake -DBENCH_MODE=ON  ..    → 启用计时，每 N 帧打印报告 + CSV
 *   cmake -DBENCH_MODE=OFF ..    → 完全编译剔除，零指令开销
 */

#ifdef BENCH_MODE

#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iostream>
#include <mutex>
#include <cmath>

using namespace std::chrono;

struct PerfRecord {
    std::string name;
    double      total_ms = 0;
    int         count    = 0;
    double      min_ms   = 1e9;
    double      max_ms   = 0;
    std::vector<double> samples;
};

class PerfTimer {
public:
    // ── 开始计时 → 返回时间戳 (ns) ──────────────────────────────────
    static inline uint64_t start() {
        return steady_clock::now().time_since_epoch().count();
    }

    // ── 结束计时 → 记录耗时 (ms) ────────────────────────────────────
    static inline double stop(const std::string& name, uint64_t start_ns) {
        uint64_t now = steady_clock::now().time_since_epoch().count();
        double ms = (now - start_ns) / 1'000'000.0;
        record(name, ms);
        return ms;
    }

    // ── RAII 作用域计时器 ──────────────────────────────────────────
    struct Scoped {
        std::string name;
        uint64_t    t0;
        Scoped(const std::string& n) : name(n), t0(PerfTimer::start()) {}
        ~Scoped() { PerfTimer::stop(name, t0); }
    };

    // ── 打印表格报告 ───────────────────────────────────────────────
    static void report(const std::string& title = "PERF REPORT") {
        std::lock_guard<std::mutex> lk(mutex_);
        printf("\n");
        printf("╔══════════════════════════════════════════════════════════════════╗\n");
        printf("║  %-62s║\n", title.c_str());
        printf("╠══════════════════════╤═══════╤════════╤════════╤════════╤════════╣\n");
        printf("║ Stage                │ count │  avg   │  min   │  max   │  p99   ║\n");
        printf("╟──────────────────────┼───────┼────────┼────────┼────────┼────────╢\n");

        for (const auto& pair : records_) {
            const auto& rec = pair.second;
            if (rec.count == 0) continue;
            double avg = rec.total_ms / rec.count;
            double p99 = percentile(rec.samples, 0.99);
            printf("║ %-20s │ %5d │ %6.2f │ %6.2f │ %6.2f │ %6.2f ║\n",
                   rec.name.c_str(), rec.count, avg, rec.min_ms, rec.max_ms, p99);
        }
        printf("╚══════════════════════╧═══════╧════════╧════════╧════════╧════════╝\n");
    }

    // ── 写 CSV 文件 ─────────────────────────────────────────────────
    static void write_csv(const std::string& path) {
        std::lock_guard<std::mutex> lk(mutex_);
        std::ofstream f(path);
        f << "stage,count,avg_ms,min_ms,max_ms,p50_ms,p99_ms,p999_ms,total_ms\n";
        for (auto& pair : records_) {
            auto& rec = pair.second;
            if (rec.count == 0) continue;
            std::sort(rec.samples.begin(), rec.samples.end());
            double avg = rec.total_ms / rec.count;
            double p50 = percentile(rec.samples, 0.50);
            double p99 = percentile(rec.samples, 0.99);
            double p999 = percentile(rec.samples, 0.999);
            f << rec.name << "," << rec.count << "," << avg
              << "," << rec.min_ms << "," << rec.max_ms
              << "," << p50 << "," << p99 << "," << p999
              << "," << rec.total_ms << "\n";
        }
        f.close();
        std::cout << "[Perf] CSV written to " << path << std::endl;
    }

private:
    static inline void record(const std::string& name, double ms) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto& r = records_[name];
        r.name = name;
        r.total_ms += ms;
        r.count++;
        if (r.count == 1) {
            r.min_ms = r.max_ms = ms;
        } else {
            r.min_ms = std::min(r.min_ms, ms);
            r.max_ms = std::max(r.max_ms, ms);
        }
        r.samples.push_back(ms);
    }

    static double percentile(const std::vector<double>& sorted, double p) {
        if (sorted.empty()) return 0.0;
        size_t idx = std::min(
            static_cast<size_t>(sorted.size() * p),
            sorted.size() - 1);
        return sorted[idx];
    }

    static inline std::map<std::string, PerfRecord> records_;
    static inline std::mutex mutex_;
};

// ── 便捷宏：在函数/代码块开头放一行即可 ───────────────────────────
#define PERF_SCOPE(name) \
    PerfTimer::Scoped _perf_scoped_##__COUNTER__(name)

#define PERF_REPORT(title) \
    PerfTimer::report(title)

#define PERF_CSV(path) \
    PerfTimer::write_csv(path)

#else  // ── BENCH_MODE OFF: 全部编译剔除 ────────────────────────────

#define PERF_SCOPE(name)       ((void)0)
#define PERF_REPORT(title)     ((void)0)
#define PERF_CSV(path)         ((void)0)

#endif
