#include "mytemplate.h"
#include "parameters.h"

mytemplate::mytemplate() {
    std::cout << "template object created\n";
}

void mytemplate::mytemplateMethod(int hue) {
    /*std::cout << "template cislo " << hue << "\n";

    parameters* kokot = parameters::GetInstance();
    std::cout << kokot << " kokot \n";
    parameters* pams = parameters::GetInstance();
    std::cout << "template parameter " << pams->param1 << "\n";
    pams->param1 = 80;
    std::cout << "template parameter2 " << pams->param1 << "\n";*/
}

mytemplate::~mytemplate() {
    std::cout << "template object destroyed\n";
}

