#pragma once
#include <iostream>
#include <iomanip>
#include <fstream>

#include <memory>

#include <string>
#include <vector>
#include <deque>

#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>

// Dependencies
#include <opencv2/opencv.hpp>



template<typename T>
using PairQue = std::deque<std::pair<std::string, T> >;

/**
 * @brief Save Queue
 * @details This class is used to save data in a queue. Each thread has its own queue, and the data will be saved in the queue in a round-robin manner.
 * @tparam T Data type to be saved. Avoid using non-copyable data type and pointer.
 * @note The function _saveCbFunc() need to be sepcialized to save custom data type.
 */
template<typename T>
class SaveQueue
{
private:
    std::deque<PairQue<T> > totalPairQue_;// Each thread's PairQue.

    std::vector<std::thread> thVec_;// Store threads.
    std::vector<std::mutex> queMutexVec_;// Each queue's mutex.
    std::vector<std::condition_variable> queCVVec_;// Each queue's condition variable.
    size_t thNum_;// Number of threads.

    std::atomic<size_t> thSelect_;// Thread selection.
    std::mutex thSelectMutex_;// Thread selection mutex.
    std::atomic<bool> availableF_;// Push available flag.
    std::atomic<bool> exitF_;// Exit flag.

private:
    /**
     * @brief Save thread function.
     * @details Each thread will wait for the condition variable to be notified, and then save the data in the queue.
     * @param[in] queID Thread ID.
     */
    void _saveTh(size_t queID)
    {
        printf("[SaveQueue::_saveTh] Thread %ld started.\n", queID);
        std::unique_lock<std::mutex> locker(this->queMutexVec_[queID], std::defer_lock);
        while (!this->exitF_)
        {
            locker.lock();
            this->queCVVec_[queID].wait(locker);
            PairQue<T> tmp(std::make_move_iterator(this->totalPairQue_[queID].begin()), 
                            std::make_move_iterator(this->totalPairQue_[queID].end()));
            this->totalPairQue_[queID].clear();
            locker.unlock();
            if (tmp.size() <= 0)
                continue;
            this->_saveCbFunc(tmp);
        }

        // Check if there is any data left
        locker.lock();
        PairQue<T> tmp(std::make_move_iterator(this->totalPairQue_[queID].begin()), 
                        std::make_move_iterator(this->totalPairQue_[queID].end()));
        this->totalPairQue_[queID].clear();
        locker.unlock();
        if (tmp.size() > 0)
            this->_saveCbFunc(tmp);
        printf("[SaveQueue::_saveTh] Thread %ld exited.\n", queID);
    }

    /**
     * @brief Save callback function.
     * @details This function need to be specialized to save data.
     * @param[in] queue Queue containing <fileName, T> pair.
     */
    void _saveCbFunc(PairQue<T>& queue)
    {
        printf("[SaveQueue::_saveCbFunc] Function not override.\n");
    }

    /**
     * @brief Get the save queue ID.
     * @details This function will return the save queue ID in a round-robin manner.
     * @return size_t Save queue ID.
     */
    size_t _getSaveQueID()
    {
        std::lock_guard<std::mutex> locker(this->thSelectMutex_);
        this->thSelect_ = (++this->thSelect_) % this->thNum_;
        return this->thSelect_;
    }

public:
    /**
     * @brief Construct a new Save Queue object
     * @param[in] thNum Number of threads.
     */
    SaveQueue(size_t thNum = 1) : thSelect_(0), availableF_(true), exitF_(false)
    {
        this->thNum_ = thNum;
        this->queMutexVec_ = std::vector<std::mutex>(thNum);
        this->queCVVec_ = std::vector<std::condition_variable>(thNum);
        this->totalPairQue_ = std::deque<PairQue<T>>(thNum);

        for (size_t i = 0; i < thNum; i++)
            this->thVec_.emplace_back(&SaveQueue::_saveTh, this, i);
    }

    ~SaveQueue()
    {
        this->close();
    }

    /**
     * @brief Push data to the save queue.
     * @param[in] fileName File name.
     * @param[in] element Data to be saved.
     * @note If the push is not available, the data will not be saved.
     */
    void push(const std::string& fileName, const T& element)
    {
        if (this->exitF_ || !this->availableF_)
            return;
        auto id = _getSaveQueID();
        std::unique_lock<std::mutex> locker(this->queMutexVec_[id], std::defer_lock);
        locker.lock();
        this->totalPairQue_[id].emplace_back(fileName, element);
        this->queCVVec_[id].notify_all();
        locker.unlock();
    }

    /**
     * @brief Get the size of the save queue.
     * @return std::vector<size_t> Size of each save queue.
     */
    std::vector<size_t> getSize()
    {
        std::vector<size_t> ret(thNum_, 0);
        for (size_t i = 0; i < this->thNum_; i++)
        {
            std::lock_guard<std::mutex> locker(this->queMutexVec_[i]);
            ret[i] = this->totalPairQue_[i].size();
        }
        return ret;
    }

    void shrink_to_fit()
    {
        if (this->exitF_)
            return;
        for (size_t i = 0; i < this->thNum_; i++)
        {
            std::lock_guard<std::mutex> locker(this->queMutexVec_[i]);
            //this->totalPairQue_[i].shrink_to_fit();
            PairQue<T>(this->totalPairQue_[i]).swap(this->totalPairQue_[i]);
        }
    }

    /**
     * @brief Close the save queue.
     * @details This function will close the save queue and wait for all threads to exit.
     * @note This function will be called automatically when the object is destroyed.
     */
    void close()
    {
        if (this->exitF_)
            return;
        this->exitF_ = true;
        std::this_thread::sleep_for(100ms);

        for (int i = 0; i < this->thNum_; i++)
        {
            this->queCVVec_[i].notify_all();// Notify all threads to exit.
            this->thVec_[i].join();
        }
    }

    /**
     * @brief Enable push.
     * @param[in] flag Enable or disable push.
     * @note If the push is not available, the data will not be saved.
     */
    void enablePush(bool flag)
    {
        this->availableF_ = flag;
    }
};

/**
 * @brief Specialization of SaveQueue for cv::Mat data type.
 * @details This specialization will save cv::Mat data to file.
 * @param[in] queue Queue containing <fileName, cv::Mat> pair.
 */
template<>
void SaveQueue<cv::Mat>::_saveCbFunc(PairQue<cv::Mat>& queue)
{
    for (const auto& [fileName, data] : queue)
    {
        cv::imwrite(fileName, data);
    }
}
