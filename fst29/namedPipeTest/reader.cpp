#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
using namespace std;

int main()
{

    string fifo_path = "/home/pi/fst29/namedPipeTest/test";
    auto mkfifo(fifo_path); //, 0666);
    std::ifstream fifo;
    fifo.open(fifo_path, ifstream::in);
    if (!fifo.is_open())
    {
        std::cout << " error : cannot open file " << std ::endl;
        return 1;
    }
    std::cout << " file open " << std ::endl;
    std::string line;
    bool done = false;
    while (!done)
    {
        while (std::getline(fifo, line))
        {
            cout << line << endl;
            /* do stuff with line */
        }
        if (fifo.eof())
        {
            fifo.clear(); // Clear the EOF bit to enable further reading
        }
        else
        {
            done = true;
        }
    }
    return 0;
}