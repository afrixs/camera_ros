#include "DispatchQueue.h"
#include <cstdint>
#include <cstdio>

namespace rtv_utils {

DispatchQueue::DispatchQueue(std::string name, size_t thread_cnt, bool process_only_latest) :
	name_(name), threads_(thread_cnt), process_only_latest_(process_only_latest)
{
	printf("Creating dispatch queue: %s\n", name.c_str());
	printf("Dispatch threads: %zu\n", thread_cnt);

	for(size_t i = 0; i < threads_.size(); i++)
	{
		threads_[i] = std::thread(&DispatchQueue::dispatch_thread_handler, this);
	}
}

DispatchQueue::~DispatchQueue()
{
	printf("Destructor: Destroying dispatch threads...\n");

	// Signal to dispatch threads that it's time to wrap up
	std::unique_lock<std::mutex> lock(lock_);
	quit_ = true;
	lock.unlock();
	cv_.notify_all();

	// Wait for threads to finish before we exit
	for(size_t i = 0; i < threads_.size(); i++)
	{
		if(threads_[i].joinable())
		{
			printf("Destructor: Joining thread %zu until completion\n", i);
			threads_[i].join();
		}
	}
}

void DispatchQueue::dispatch(const fp_t& op)
{
	std::unique_lock<std::mutex> lock(lock_);
	q_.push(op);

	// Manual unlocking is done before notifying, to avoid waking up
    // the waiting thread only to block again (see notify_one for details)
	lock.unlock();
	cv_.notify_all();
}

void DispatchQueue::dispatch(fp_t&& op)
{
	std::unique_lock<std::mutex> lock(lock_);
	q_.push(std::move(op));

	// Manual unlocking is done before notifying, to avoid waking up
    // the waiting thread only to block again (see notify_one for details)
	lock.unlock();
	cv_.notify_all();
}

void DispatchQueue::dispatch_thread_handler(void)
{
	std::unique_lock<std::mutex> lock(lock_);

	do {
		//Wait until we have data or a quit signal
		cv_.wait(lock, [this]{
			return (q_.size() || quit_);
		});

		//after wait, we own the lock
		if(!quit_ && q_.size())
		{
            
			auto op = std::move(q_.front());
			q_.pop();
            if (process_only_latest_) {
                while (!quit_ && q_.size()) {
                    op = std::move(q_.front());
                    q_.pop();
                }
            }


			//unlock now that we're done messing with the queue
			lock.unlock();

			op();

			lock.lock();
		}
	} while (!quit_);
}

}