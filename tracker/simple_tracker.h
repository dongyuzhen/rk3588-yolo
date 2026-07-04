#pragma once

/**
 * @file simple_tracker.h
 * @brief 轻量 IoU 目标追踪器（纯 C++17，无第三方依赖）
 *
 * 设计思路：
 *  - 贪心 IoU 匹配：对每帧检测框与活跃 Track 按 IoU 降序贪心配对
 *  - 速度预测：线性外推上一帧位移，提升快速运动匹配成功率
 *  - 生命周期：hit_count ≥ HIT_CONFIRM 才"确认"输出，miss_count > MAX_MISS 则删除
 *  - 告警增益：track_id 作为人员身份锚点，配合报警引擎实现 ID 级别冷却去重
 */

#include <algorithm>
#include <cstring>
#include <limits>
#include <vector>

#include "../yolo/post_process.h"  // detect_result_group_t, detect_result_t, box_p

// ─── 追踪结果：在 detect_result_t 基础上附加 track_id ───────────────────────
struct TrackedResult {
    detect_result_t det;  // 原始检测结果（label、conf、box）
    int track_id;         // 稳定目标 ID（跨帧不变）
    int hit_count;        // 累计命中帧数（可用于置信度评估）
};

// ─── 内部 Track 状态 ──────────────────────────────────────────────────────────
struct Track {
    int   id;
    box_p box;           // 当前帧位置
    box_p prev_box;      // 上帧位置（用于速度预测）
    box_p smooth_box;    // EMA 平滑后的位置（防抖）
    int   miss_count;    // 连续未匹配帧数
    int   hit_count;     // 累计命中帧数
    char  label[32];

    // 线性速度外推，预测本帧可能位置
    box_p predict() const {
        int dx = box.xmin - prev_box.xmin;
        int dy = box.ymin - prev_box.ymin;
        return {box.xmin + dx, box.ymin + dy,
                box.xmax + dx, box.ymax + dy};
    }
};

// ─── SimpleTracker ────────────────────────────────────────────────────────────
class SimpleTracker {
public:
    // 超参（均可按实际场景调整）
    static constexpr float IOU_THRESHOLD  = 0.25f;  // 低于此值不匹配（移动快时可调低）
    static constexpr int   MAX_MISS       = 8;       // 连续未匹配帧数上限
    static constexpr int   HIT_CONFIRM    = 2;       // 命中≥2帧才输出（过滤单帧误检）

    /**
     * @brief 核心更新接口：输入当前帧检测结果，返回带 track_id 的追踪结果
     * @param group  YOLO 后处理输出的检测结果组
     * @return       带 track_id 的追踪结果列表（仅含已确认目标）
     */
    std::vector<TrackedResult> update(const detect_result_group_t& group) {
        int n_det = group.box_count;
        int n_trk = static_cast<int>(tracks_.size());

        // ── Step 1：用速度预测更新每条 Track 的预期位置 ─────────────────────
        std::vector<box_p> predicted(n_trk);
        for (int i = 0; i < n_trk; ++i)
            predicted[i] = tracks_[i].predict();

        // ── Step 2：构建 IoU 矩阵 [det × trk] ───────────────────────────────
        // iou_mat[d][t] = IoU(det_d, predicted_t)
        std::vector<std::vector<float>> iou_mat(n_det, std::vector<float>(n_trk, 0.f));
        for (int d = 0; d < n_det; ++d)
            for (int t = 0; t < n_trk; ++t)
                iou_mat[d][t] = compute_iou(group.result[d].box, predicted[t]);

        // ── Step 3：贪心匹配（按 IoU 降序，每个 det/trk 最多匹配一次） ───────
        std::vector<int> det_match(n_det, -1);   // det → trk index
        std::vector<bool> trk_used(n_trk, false);

        // 收集所有 (iou, d, t) 并排序
        struct Triple { float iou; int d, t; };
        std::vector<Triple> pairs;
        pairs.reserve(n_det * n_trk);
        for (int d = 0; d < n_det; ++d)
            for (int t = 0; t < n_trk; ++t)
                if (iou_mat[d][t] >= IOU_THRESHOLD)
                    pairs.push_back({iou_mat[d][t], d, t});

        std::sort(pairs.begin(), pairs.end(),
                  [](const Triple& a, const Triple& b){ return a.iou > b.iou; });

        for (auto& p : pairs) {
            if (det_match[p.d] == -1 && !trk_used[p.t]) {
                det_match[p.d] = p.t;
                trk_used[p.t]  = true;
            }
        }

        // ── Step 4：更新匹配的 Track ─────────────────────────────────────────
        for (int d = 0; d < n_det; ++d) {
            int t = det_match[d];
            if (t < 0) continue;
            Track& tr = tracks_[t];
            tr.prev_box   = tr.box;
            tr.box        = group.result[d].box;

            // EMA (Exponential Moving Average) 平滑处理
            // alpha 值越小平滑越强，越大越跟随原始检测
            float alpha = 0.6f;
            tr.smooth_box.xmin = static_cast<int>(alpha * tr.box.xmin + (1.0f - alpha) * tr.smooth_box.xmin);
            tr.smooth_box.ymin = static_cast<int>(alpha * tr.box.ymin + (1.0f - alpha) * tr.smooth_box.ymin);
            tr.smooth_box.xmax = static_cast<int>(alpha * tr.box.xmax + (1.0f - alpha) * tr.smooth_box.xmax);
            tr.smooth_box.ymax = static_cast<int>(alpha * tr.box.ymax + (1.0f - alpha) * tr.smooth_box.ymax);

            tr.miss_count = 0;
            tr.hit_count++;
            std::strncpy(tr.label, group.result[d].label, 31);
            tr.label[31] = '\0';
        }

        // ── Step 5：未匹配的 Track → miss_count++ ───────────────────────────
        for (int t = 0; t < n_trk; ++t) {
            if (!trk_used[t])
                tracks_[t].miss_count++;
        }

        // ── Step 6：未匹配的检测框 → 新建 Track ─────────────────────────────
        for (int d = 0; d < n_det; ++d) {
            if (det_match[d] != -1) continue;
            Track tr;
            tr.id         = next_id_++;
            tr.box        = group.result[d].box;
            tr.prev_box   = group.result[d].box;  // 初始速度为0
            tr.smooth_box = group.result[d].box;  // 初始平滑框
            tr.miss_count = 0;
            tr.hit_count  = 1;
            std::strncpy(tr.label, group.result[d].label, 31);
            tr.label[31]  = '\0';
            tracks_.push_back(tr);
        }

        // ── Step 7：删除失效 Track（miss 超限） ──────────────────────────────
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [](const Track& tr){ return tr.miss_count > MAX_MISS; }),
            tracks_.end());

        // ── Step 8：构建输出（仅输出已确认 + 本帧匹配的 Track） ──────────────
        std::vector<TrackedResult> output;
        output.reserve(n_det);

        // 重建 det → track_id 映射（经过删除后需重新找）
        for (int d = 0; d < n_det; ++d) {
            // 找到 box 匹配的 Track（刚才更新过）
            for (const Track& tr : tracks_) {
                if (box_equal(tr.box, group.result[d].box) &&
                    std::strncmp(tr.label, group.result[d].label, 31) == 0 &&
                    tr.hit_count >= HIT_CONFIRM) {
                    TrackedResult res;
                    res.det      = group.result[d];
                    res.det.box  = tr.smooth_box; // ★ 输出平滑后的框替换原始跳动框
                    res.track_id = tr.id;
                    res.hit_count= tr.hit_count;
                    output.push_back(res);
                    break;
                }
            }
        }
        return output;
    }

    // 当前活跃 Track 数（调试用）
    int active_count() const { return static_cast<int>(tracks_.size()); }

private:
    std::vector<Track> tracks_;
    int next_id_ = 1;

    // ── IoU 计算 ─────────────────────────────────────────────────────────────
    static float compute_iou(const box_p& a, const box_p& b) {
        int ix1 = std::max(a.xmin, b.xmin);
        int iy1 = std::max(a.ymin, b.ymin);
        int ix2 = std::min(a.xmax, b.xmax);
        int iy2 = std::min(a.ymax, b.ymax);

        if (ix2 <= ix1 || iy2 <= iy1) return 0.f;

        float inter = static_cast<float>((ix2 - ix1) * (iy2 - iy1));
        float area_a = static_cast<float>((a.xmax - a.xmin) * (a.ymax - a.ymin));
        float area_b = static_cast<float>((b.xmax - b.xmin) * (b.ymax - b.ymin));
        float uni = area_a + area_b - inter;
        return (uni <= 0.f) ? 0.f : inter / uni;
    }

    // 精确 box 相等比较（用于 Step 8 回查）
    static bool box_equal(const box_p& a, const box_p& b) {
        return a.xmin == b.xmin && a.ymin == b.ymin &&
               a.xmax == b.xmax && a.ymax == b.ymax;
    }
};
