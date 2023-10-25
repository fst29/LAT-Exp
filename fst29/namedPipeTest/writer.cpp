#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
using namespace std;

int main()
{
    std::ofstream fifo;
    fifo.open("/home/pi/fst29/namedPipeTest/test", ios::out);
    if (!fifo.is_open())
    {
        std::cout << " error : cannot open file " << std ::endl;
        return 1;
    }
    std::cout << " file open " << std ::endl;
    fifo.close();
    std::string line;
    while (line.compare("exit") != 0)
    {

        std::getline(cin, line);
        fifo.open("/home/pi/fst29/namedPipeTest/test", ios::out);
        fifo << line;
        /* do stuff with line */
        fifo.close();
    }
    fifo.close();
    return 0;
}