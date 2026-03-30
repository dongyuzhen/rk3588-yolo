#ifndef SAFEQUEUE_H
#define SAFEQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
using namespace std;

template<typename T>
class SafeQueue
{

public:
    SafeQueue(size_t maxSize_in) : maxSize(maxSize_in){}
    ~SafeQueue(){};

    //插入队列
    void enqueue(const T &t )
    {
        unique_lock<mutex> lock(m);
        cond_not_full.wait(lock, [this]{ return stop_flag || q.size() < maxSize; });
        if (stop_flag) {
            return;
        }

        q.push(t);
        cond_not_empty.notify_one();
    }

    // 从队列弹出
    bool dequeue(T &t)
    {
        unique_lock<mutex> lock(m);
        cond_not_empty.wait(lock,[this] { return stop_flag || !q.empty(); });
        if (q.empty()) {
            // stop_flag==true 且队列为空，或异常唤醒后为空
            return false;
        }

        t = q.front();
        q.pop();

        cond_not_full.notify_one();
        return true;
    }
    // 调用此函数让所有阻塞线程退出
    void stop() {
        std::unique_lock<std::mutex> lock(m);
        stop_flag = true;
        cond_not_empty.notify_all();
        cond_not_full.notify_all();
    }

    // 返回队列是否为空
    bool empty()
    {
        unique_lock<mutex> lock(m);
        return q.empty();
    }

    // 返回队列当前元素数量
    size_t size()
    {
        unique_lock<mutex> lock(m);
        return q.size();
    }
    private:
    bool stop_flag = false;
    queue<T> q;
    mutable mutex m;
    std::condition_variable cond_not_empty;
    std::condition_variable cond_not_full;
    size_t maxSize;
};

#endif
