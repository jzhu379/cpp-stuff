#include "KeyedPriorityQueue.hpp"

#include <cassert>
#include <iostream>
#include <string>

int main() {
    KeyedPriorityQueue<std::string, unsigned int> kpq;

    assert(kpq.isEmpty());
    assert(kpq.size() == 0);

    kpq.push("abc", 12);
    assert(!kpq.isEmpty());
    assert(kpq.size() == 1);

    assert(kpq.getPriority("abc") == 12);
    assert(kpq.top().first == "abc");
    assert(kpq.top().second == 12);

    assert(kpq.containsKey("abc"));
    kpq.removeKey("abc");
    assert(!kpq.containsKey("abc"));

    std::cout << "SUCCESS" << std::endl;

    return 0;
}