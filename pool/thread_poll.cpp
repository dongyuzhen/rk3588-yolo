#include "thread_poll.h"

ThreadPoll::ThreadPoll(const char* model_path, int num_threads)
{
    // 这里可以做一些通用初始化，比如 run_flag=true
    run_flag = true;
    // 初始化：加载模型，启动线程
    init(model_path, num_threads);
}

ThreadPoll::~ThreadPoll()
{
    // 在ThreadPoll析构函数添加
    std::cout << "Remaining tasks: " << tasks.size() << std::endl;

    // 通知线程退出
    run_flag = false;
    // 唤醒所有等待条件，让 worker() 能跳出循环
    condition.notify_all();

    // 等待线程结束
    for(auto& t : threads)
    {
        if(t.joinable())
        {
            t.join();
        }
    }
    // 这里你也可以释放模型等资源
    std::cout << "ThreadPoll destroyed.\n";
}

void ThreadPoll::init(const char* model_path, int num_threads)
{

    if(num_threads <= 0) num_threads = 1; // 保底
    // 比如按照 num_threads 个 Yolov5s
    // 也可以根据需求只创建几个再共享
    for(int i = 0; i < num_threads; i++)
    {
        auto yolo = std::make_shared<Yolov5s>(model_path, i % 3);
        yolo_group.emplace_back(yolo);
    }

    // 启动 num_threads 个工作线程
    for(int i = 0; i < num_threads; i++)
    {
        threads.emplace_back(&ThreadPoll::worker, this, i);
    }
}

// worker：只对 tasks 这个队列做等待、取出、执行
void ThreadPoll::worker(int id)
{
    // 取到专属的yolo实例
    std::shared_ptr<Yolov5s> yolo = yolo_group[id];
    std::cout << "worker线程启动, id=" << id << "\n";
    while(run_flag)
    {
        std::packaged_task<ProcessResult(std::shared_ptr<Yolov5s>)> current_task;
        {
            // 阻塞等待队列内有任务，或等到退出信号
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this]
            {
                return (!tasks.empty() || !run_flag);
            });

            if(!run_flag)
            {
                std::cout << "worker " << id << " 下班！\n";
                break;
            }

            current_task = std::move(tasks.front());
            tasks.pop();
        }

        if(current_task.valid())
        {
            current_task(yolo);
        }
    }
    std::cout << "Worker " << id << " exited, remaining tasks: " << tasks.size() << std::endl;
}

// 新的方法：往 tasks 里塞任务，并用 std::future<ProcessResult> 返回结果
//cv::Mat 来源于解码，假设换成cmafd
//std::future<ProcessResult> ThreadPoll::submit_task_async(int index, cv::Mat img)    
std::future<ProcessResult> ThreadPoll::submit_task_async(int index, int dmabuf_fd)   //改成传入v4l2的fd
{
    std::packaged_task<ProcessResult(std::shared_ptr<Yolov5s>)> task([index, dmabuf_fd](std::shared_ptr<Yolov5s> yolo)
    {
        ProcessResult result;
        try
        {
            detect_result_group_t detections;
            int ret = yolo->inference_image(dmabuf_fd, detections);
            if (ret != 0) {
                result.error_msg = "inference_image failed, ret=" + std::to_string(ret);
                result.success = false;
            } else {
                result.detection_results = detections;
                result.success = true;
            }
        }
        catch(const std::exception& e)
        {
            result.error_msg = e.what();
            result.success = false;
        }
        return result;
    });

    std::future<ProcessResult> future = task.get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        tasks.emplace(std::move(task));
    }
    condition.notify_one();
    return future;
}
