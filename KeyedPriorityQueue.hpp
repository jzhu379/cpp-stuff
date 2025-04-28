#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * @brief Priority queue that associates each priority with a hashable key.
 * Priorities are added with a user-defined key, and priorities can be modified
 * or removed using the key. Maximum priorities are placed at the top by
 * default.
 * @details pop, push, getPriority, removeKey have O(log(n)) time complexity,
 * assuming comparison takes O(1) time complexity.
 * @param Key Key type.
 * @param Priority Priority type.
 * @param Compare Comparison object that can compare priorities for ordering.
 * The default is std::less<Priority>.
 * @param Hash The hash for key types.
 * @param KeyEqual Object for determining equality between keys.
 */
template <class Key, class Priority, class Compare = std::less<Priority>,
          class Hash = std::hash<Key>, class KeyEqual = std::equal_to<Key>>
class KeyedPriorityQueue {
  public:
    /**
     * @brief Default constructor creates a maximum priority queue.
     */
    KeyedPriorityQueue() : cmp_(Compare{}) {}

    /**
     * @brief Constructor that uses a custom comparison function.
     * @param cmp A function object for custom comparison that must be in the
     * form of (const Priority &a, const Priority &b) -> bool, where the return
     * value represents a < b.
     * @example Using a cmp of (const Priority& a, const Priority& b) { return a
     * > b; } would specify a minimum priority queue.
     */
    KeyedPriorityQueue(
        const std::function<bool(const Priority &, const Priority &)> &cmp)
        : cmp_(cmp) {}

    /**
     * @brief Returns boolean indicating if priority queue is empty.
     */
    bool isEmpty() const { return priorities_.empty(); }

    /**
     * @brief Return size of priority queue.
     */
    std::size_t size() const { return priorities_.size(); }

    /**
     * @brief Delete all entries currently in the priority queue.
     */
    void clear() {
        priorities_.clear();
        keyIdx_.clear();
    }

    /**
     * @brief Return reference to top of priority queue, returning a (key,
     * priority) pair. Throws if empty.
     */
    const std::pair<Key, Priority> &top() const {
        if (isEmpty()) {
            throw std::out_of_range("queue is empty");
        }

        return priorities_[0];
    }

    /**
     * @brief Insert priority if key does not exist; otherwise, the existing
     * priority associated with the key is updated.
     * @param key Key to insert.
     * @param priority Priority to insert.
     */
    void push(const Key &key, const Priority &priority) {
        if (!containsKey(key)) {
            priorities_.emplace_back(key, priority);

            std::size_t idx = size() - 1;
            keyIdx_[key] = idx;
            moveUpwards(idx);

            return;
        }

        std::size_t idx = keyIdx_[key];
        Priority oldPriority = std::move(priorities_[idx].second);
        priorities_[idx].second = priority;

        if (cmp_(oldPriority, priority)) {
            moveUpwards(idx);
        } else {
            moveDownwards(idx);
        }
    }

    /**
     * @brief Move priority version of push().
     * @param key Key to insert.
     * @param priority Priority to insert, taking a r-value reference.
     */
    void push(const Key &key, Priority &&priority) {
        if (!containsKey(key)) {
            priorities_.emplace_back(key, std::move(priority));

            std::size_t idx = size() - 1;
            keyIdx_[key] = idx;
            moveUpwards(idx);

            return;
        }

        std::size_t idx = keyIdx_[key];
        Priority oldPriority = std::move(priorities_[idx].second);
        priorities_[idx].second = std::move(priority);

        if (cmp_(oldPriority, priorities_[idx].second)) {
            moveUpwards(idx);
        } else {
            moveDownwards(idx);
        }
    }

    /**
     * @brief Remove the top of the priority queue, if it exists; do nothing
     * otherwise.
     */
    void pop() {
        if (isEmpty()) {
            return;
        }

        if (size() == 1) {
            clear();
            return;
        }

        keyIdx_.erase(priorities_[0].first);

        priorities_[0].first = std::move(priorities_[size() - 1].first);
        priorities_[0].second = std::move(priorities_[size() - 1].second);
        priorities_.pop_back();

        keyIdx_[priorities_[0].first] = 0;

        moveDownwards(0);
    }

    /**
     * @brief Returns boolean indicating if a key exists in the priority queue.
     * @param key Key to check.
     */
    bool containsKey(const Key &key) const { return keyIdx_.count(key); }

    /**
     * @brief Return reference to the priority associated with the key. Throws
     * if key does not exist.
     * @param key Key used to obtain associated priority.
     */
    const Priority &getPriority(const Key &key) const {
        if (!containsKey(key)) {
            throw std::out_of_range("key does not exist");
        }

        return priorities_[keyIdx_.at(key)].second;
    }

    /**
     * @brief Remove the priority associated with the key, if the key exists.
     * Does nothing otherwise.
     * @param key Key used to remove associated priority.
     */
    void removeKey(const Key &key) {
        if (!containsKey(key)) {
            return;
        }

        std::size_t idx = keyIdx_[key];
        if (idx == size() - 1) {
            keyIdx_.erase(key);
            priorities_.pop_back();
            return;
        }

        Priority oldPriority = std::move(priorities_[idx].second);

        priorities_[idx].first = std::move(priorities_[size() - 1].first);
        priorities_[idx].second = std::move(priorities_[size() - 1].second);
        priorities_.pop_back();

        keyIdx_[priorities_[idx].first] = idx;
        keyIdx_.erase(key);

        if (cmp_(oldPriority, priorities_[idx].second)) {
            moveUpwards(idx);
        } else {
            moveDownwards(idx);
        }
    }

    /**
     * @brief Const iterators that expose the internal keys and priorities.
     */
    typename std::vector<std::pair<Key, Priority>>::const_iterator
    begin() const {
        return priorities_.begin();
    }
    typename std::vector<std::pair<Key, Priority>>::const_iterator end() const {
        return priorities_.end();
    }

  private:
    /**
     * @brief Comparison function used to determine priority.
     */
    const std::function<bool(const Priority &, const Priority &)> cmp_;

    /**
     * @brief Vector containing elements of the priority queue.
     */
    std::vector<std::pair<Key, Priority>> priorities_;

    /**
     * @brief Mapping of {key : idx}, where idx represents the index location of
     * the entry in the priorities_ vector;
     */
    std::unordered_map<Key, std::size_t, Hash, KeyEqual> keyIdx_;

    /**
     * @brief If necessary, move the entry at idx upwards after a priority
     * increase.
     */
    void moveUpwards(std::size_t idx) {
        int parentIdx = (idx - 1) / 2;
        while (idx >= 1 &&
               cmp_(priorities_[parentIdx].second, priorities_[idx].second)) {
            keyIdx_[priorities_[parentIdx].first] = idx;
            keyIdx_[priorities_[idx].first] = parentIdx;

            std::swap(priorities_[idx], priorities_[parentIdx]);

            idx = parentIdx;
            parentIdx = (idx - 1) / 2;
        }
    }

    /**
     * @brief If necessary, move the entry at idx downwards after a priority
     * decrease.
     */
    void moveDownwards(std::size_t idx) {
        std::size_t maxPriorityIdx = getChildrenMax(idx);
        while (maxPriorityIdx != idx) {
            keyIdx_[priorities_[idx].first] = maxPriorityIdx;
            keyIdx_[priorities_[maxPriorityIdx].first] = idx;

            std::swap(priorities_[idx], priorities_[maxPriorityIdx]);

            idx = maxPriorityIdx;
            maxPriorityIdx = getChildrenMax(idx);
        }
    }

    /**
     * @brief Helper function for moveDownwards.
     * @details Return the index of the entry with the largest priority, out of
     * the nodes {idx, idx.left, idx.right}, where the left and right nodes of
     * the heap may be logically null.
     */
    std::size_t getChildrenMax(std::size_t idx) const {
        std::size_t maxPriorityIdx = idx;

        std::size_t leftIdx = 2 * idx + 1;
        if (leftIdx < size() && cmp_(priorities_[maxPriorityIdx].second,
                                     priorities_[leftIdx].second)) {
            maxPriorityIdx = leftIdx;
        }

        std::size_t rightIdx = 2 * idx + 2;
        if (rightIdx < size() && cmp_(priorities_[maxPriorityIdx].second,
                                      priorities_[rightIdx].second)) {
            maxPriorityIdx = rightIdx;
        }

        return maxPriorityIdx;
    }
};
