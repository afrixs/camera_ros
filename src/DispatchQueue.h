
#ifndef __DispatchQueue__
#define __DispatchQueue__

#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <string>
#include <condition_variable>
#include <functional>

namespace rtv_utils {

class DispatchQueue {
	typedef std::function<void(void)> fp_t;

public:
	DispatchQueue(std::string name, size_t thread_cnt = 1, bool process_only_latest = false);
	~DispatchQueue();

	// dispatch and copy
	void dispatch(const fp_t& op);
	// dispatch and move
	void dispatch(fp_t&& op);

	// Deleted operations
	DispatchQueue(const DispatchQueue& rhs) = delete;
	DispatchQueue& operator=(const DispatchQueue& rhs) = delete;
	DispatchQueue(DispatchQueue&& rhs) = delete;
	DispatchQueue& operator=(DispatchQueue&& rhs) = delete;

private:
	std::string name_;
	std::mutex lock_;
	std::vector<std::thread> threads_;
	std::queue<fp_t> q_;
	std::condition_variable cv_;
	bool quit_ = false;
    bool process_only_latest_;

	void dispatch_thread_handler(void);
};

}

#endif