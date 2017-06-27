#include <sec/prompter.h>

#include <iostream>
#undef NDEBUG
#include <cassert>

void printMsg(void) {

    std::cout << "test message (fun) \n";

}

class Printable {

public:
    void operator()() {
        std::cout << "test message (operator) \n";
        t = 1;
    }

    void pmember() {
        std::cout << "test message (member) \n";
        t = 2;
    }

    int t = 0;

};

void consumeString(const std::string& str) {
    std::cout << "here is your string: " << str << std::endl;
}

int main(void) {

    Printable prin;

    sec::YesNoExecutePrompt p1("Do you want to execute the function?", printMsg);
    sec::YesNoExecutePrompt p2("Do you want to execute the function?", prin);                      // no side effects, it's a copy!!!
    sec::YesNoExecutePrompt p3("Do you want to execute the function?", prin, &Printable::pmember);

    sec::StringArgExecutePrompt p4("Please insert a string:", consumeString);

    sec::ConditionalPrompt p5("Do you want to execute this other function?", p4);

    sec::Prompter prompter;
    prompter.addPrompt(p1);
    prompter.addPrompt(p2);
    prompter.addPrompt(p3);
    prompter.addPrompt(p4);
    prompter.addPrompt(p5);

    prompter.executeAll();

    std::cout << prin.t << std::endl;

    return 0;

}
