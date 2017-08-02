#include <sec/sec.h>
#include <sec/events.h>

#include <iostream>
#undef NDEBUG
#include <cassert>


// class case
class EventTest {

public:
    int val = 0;

    void changeVal() {
        std::cout << "event3" << std::endl;
        val = 1;
    }

};


// function case
int funVal = 0;
void funChangeVal() {
    std::cout << "event2" << std::endl;
    funVal = 1;
}


int main(void) {

    sec::EventManager manager(100.0);

    manager.addEvent([] { std::cout << "event1" << std::endl; }, 1.0);

    manager.addEvent(funChangeVal, 2.0);

    EventTest et;
    manager.addEvent(et, &EventTest::changeVal, 3.0);

    sec::run(4.0);

    assert(funVal == 1);
    assert(et.val == 1);

    return 0;

}
