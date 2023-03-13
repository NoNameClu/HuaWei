#include <iostream>
#include <string>

#include "Command.h"
using namespace std;

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}

int main() {
    Command command;
    command.init();
    command.start();
    return 0;
}
