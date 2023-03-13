#include <iostream>
#include <string>
#include "Command.h"
using namespace std;

int main() {
    Command command;
    command.init();
    command.start();
    return 0;
}
