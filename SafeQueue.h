#ifndef SAFEQUEUE_H
#define SAFEQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SafeQueue
{
public:
    explicit SafeQueue(size_t maxSize) : max_size(maxSize) {}

    ~SafeQueue()
    {
        stop();
    }

    // 入队（阻塞，直到队列不满）
    void enqueue(T val)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]
                { return stop_flag || q.size() < max_size; });

        if (stop_flag)
            return;

        q.push(std::move(val)); // 移动语义，减少拷贝（巨大优化）
        cv.notify_one();
    }

    // 出队（阻塞，直到队列非空）
    bool dequeue(T &val)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]
                { return stop_flag || !q.empty(); });

        if (q.empty())
            return false;

        val = std::move(q.front());
        q.pop();

        cv.notify_one();
        return true;
    }

    // 带有回调处理的入队策略（如果满了，弹出最老的一帧交给回调函数处理）
    template <typename DropHandler>
    void enqueue_drop_oldest(T val, DropHandler on_drop)
    {
        T dropped_val;
        bool dropped = false;
        
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (q.size() >= max_size) {
                dropped_val = std::move(q.front()); // 取出最老的帧
                q.pop();                            // 从队列移除
                dropped = true;
            }
            q.push(std::move(val));
            cv.notify_one();
        } // 在回调前释放锁，避免死锁或降低并发性能

        // 如果发生了丢帧，再去调用回调函数并交还旧帧
        if (dropped) {
            on_drop(dropped_val);
        }
    }

    // 唤醒所有等待线程
    void stop()
    {
        std::lock_guard<std::mutex> lock(mtx);
        stop_flag = true;
        cv.notify_all();
    }

    // 【高效】只在必要时才锁
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        return q.empty();
    }


private:
    size_t max_size;
    std::queue<T> q;
    mutable std::mutex mtx;
    std::condition_variable cv; // 只需要一个！
    bool stop_flag = false;
};

#endif