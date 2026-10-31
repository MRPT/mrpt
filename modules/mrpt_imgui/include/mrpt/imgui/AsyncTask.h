/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace mrpt::imgui
{
/** Shared state between an AsyncTask and its worker function: the worker
 *  reports progress and checks for cancellation through it.
 * \ingroup mrpt_imgui_grp
 */
class AsyncTaskContext
{
 public:
  /** Reports progress in [0,1], and optionally a short status message. */
  void setProgress(double fraction, const std::string& message = {})
  {
    m_progress.store(fraction);
    if (!message.empty())
    {
      std::lock_guard<std::mutex> lck(m_msgMtx);
      m_message = message;
    }
  }
  [[nodiscard]] double progress() const { return m_progress.load(); }
  [[nodiscard]] std::string message() const
  {
    std::lock_guard<std::mutex> lck(m_msgMtx);
    return m_message;
  }

  /** Long workers should poll this and return early when true. */
  [[nodiscard]] bool cancelRequested() const { return m_cancel.load(); }
  void requestCancel() { m_cancel.store(true); }

 private:
  std::atomic<double> m_progress{0.0};
  std::atomic<bool> m_cancel{false};
  mutable std::mutex m_msgMtx;
  std::string m_message;
};

/** Runs a computation on a background thread, and lets the GUI thread poll
 *  for its result once per frame without blocking, so long operations (e.g.
 *  loading a large file) never stall the event loop.
 *
 *  Starting a new computation while one is running supersedes it: the old
 *  one is asked to cancel, and its result is discarded when it finishes.
 *
 *  \code
 *    mrpt::imgui::AsyncTask<Scene::Ptr> loader;
 *    loader.start([file](AsyncTaskContext& ctx) { return loadScene(file, ctx); });
 *    // Every frame:
 *    if (auto scene = loader.poll(); scene) { ... }
 *    if (loader.isRunning()) { ImGui::ProgressBar(loader.progress()); }
 *  \endcode
 * \ingroup mrpt_imgui_grp
 */
template <typename T>
class AsyncTask
{
 public:
  using worker_t = std::function<T(AsyncTaskContext&)>;

  AsyncTask() = default;
  ~AsyncTask() { cancel(); }

  AsyncTask(const AsyncTask&) = delete;
  AsyncTask& operator=(const AsyncTask&) = delete;

  /** Starts `fn` on a background thread (cancelling any running one). */
  void start(worker_t fn)
  {
    cancel();
    m_ctx = std::make_shared<AsyncTaskContext>();
    auto ctx = m_ctx;
    m_future = std::async(std::launch::async, [fn = std::move(fn), ctx]() { return fn(*ctx); });
    m_running = true;
    m_generation++;
  }

  /** Starts a worker that needs no progress or cancellation support. */
  void start(std::function<T()> fn)
  {
    start(worker_t([fn = std::move(fn)](AsyncTaskContext&) { return fn(); }));
  }

  [[nodiscard]] bool isRunning() const { return m_running; }

  /** Number of computations started so far (so callers can tell results
   * of different runs apart). */
  [[nodiscard]] uint64_t generation() const { return m_generation; }

  /** Progress reported by the running worker, in [0,1]. */
  [[nodiscard]] double progress() const { return m_ctx ? m_ctx->progress() : 0.0; }
  [[nodiscard]] std::string progressMessage() const
  {
    return m_ctx ? m_ctx->message() : std::string();
  }

  /** Asks the running worker to stop (it must poll cancelRequested()), and
   * forgets about it: its result, if any, is never returned by poll(). The
   * worker thread is not waited for. */
  void cancel()
  {
    if (!m_running)
    {
      return;
    }
    if (m_ctx)
    {
      m_ctx->requestCancel();
    }
    // Keep the future alive in a detached waiter, so the destructor of
    // std::future (which blocks for std::async) does not block the GUI.
    auto orphan = std::make_shared<std::future<T>>(std::move(m_future));
    std::thread([orphan]() { orphan->wait(); }).detach();
    m_running = false;
  }

  /** Like cancel(), but waits for the worker to finish, e.g. before
   * destroying data it uses. */
  void cancelAndWait()
  {
    if (!m_running)
    {
      return;
    }
    if (m_ctx)
    {
      m_ctx->requestCancel();
    }
    m_future.wait();
    m_running = false;
  }

  /** Call once per frame: returns the result exactly once, on the first
   * call after the worker finished, and nullopt otherwise. If the worker
   * threw, the exception is rethrown here. */
  std::optional<T> poll()
  {
    if (!m_running)
    {
      return std::nullopt;
    }
    if (m_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      return std::nullopt;
    }
    m_running = false;
    return m_future.get();
  }

 private:
  std::future<T> m_future;
  std::shared_ptr<AsyncTaskContext> m_ctx;
  bool m_running = false;
  uint64_t m_generation = 0;
};

}  // namespace mrpt::imgui
