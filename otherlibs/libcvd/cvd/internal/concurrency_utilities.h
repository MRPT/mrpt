#ifndef CVD_INTERNAL_CONCURRENCY_UTILITIES_H_hOOrezFKvnMkUQ
#define CVD_INTERNAL_CONCURRENCY_UTILITIES_H_hOOrezFKvnMkUQ

#include <deque>
#include <atomic>
#include <mutex>
#include <condition_variable>


namespace CVD{
	namespace Internal{

		class Semaphore
		{
			private:
				std::atomic<int> remaining;
				std::mutex m;
				std::condition_variable not_empty;



			public:
				Semaphore(int n)
					:remaining(n)
				{
				}

				bool maybe_acquire()
				{
					std::lock_guard<std::mutex> lock(m);

					if(remaining == 0)
						return false;
					else
						remaining--;
				}

				void acquire()
				{
					//Acquire the lock
					std::unique_lock<std::mutex> lock(m);

					//If there's nothing to acquire, then release the lock 
					//and wait. Note the wakeup may be spurious.
					while(remaining == 0)
						not_empty.wait(lock);

					remaining--;
				}


				void release()
				{
					std::unique_lock<std::mutex> lock(m);
					remaining++;
					not_empty.notify_one();
				}
		};


		template<class C>
			class MessageQueue
			{
				private:
					std::deque<C> data;
					std::mutex queue_mutex;
					Semaphore free_slots, used_slots;
					std::atomic<int> length;

				public:
					MessageQueue(size_t sz)
						:free_slots(sz),used_slots(0)
					{
					}


					void push(const C& a)
					{
						used_slots.release();
						free_slots.acquire();

						std::lock_guard<std::mutex> lock(queue_mutex);
						data.push_back(move(a));
						length = data.size();
					}

					void push(C&& a)
					{
						used_slots.release();
						free_slots.acquire();

						std::lock_guard<std::mutex> lock(queue_mutex);
						data.emplace_back(move(a));
						length = data.size();

					}

					bool maybe_pop(C&& ret)
					{
						if(used_slots.maybe_acquire())
						{
							free_slots.release();
							std::lock_guard<std::mutex> lock(queue_mutex);

							ret = move(data.front());
							data.pop_front();
							length = data.size();
							return true;
						}
						else
							return false;
					}


					C pop()
					{
						used_slots.acquire();
						free_slots.release();
						std::lock_guard<std::mutex> lock(queue_mutex);

						C a = move(data.front());
						data.pop_front();
						length = data.size();
						return a;
					}

					bool empty() const
					{
						return length == 0;
					}

					int get_length() const
					{
						return length;
					}
			};



	}
}



#endif

