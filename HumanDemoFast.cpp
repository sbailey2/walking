#include <iostream>

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "HumanDemo.h"

int main(int argc, char *argv[])
{

    // Initialize the demo
    CommonExampleInterface* currentDemo = 0;
    CommonExampleOptions options(0,0);
    currentDemo = HumanControlCreateFunc(options);
    currentDemo->initPhysics();
    
    // Update the demo indefinitely
    double deltaTime = 1.0/60.0;
    while(true) {
	currentDemo->stepSimulation(deltaTime);
	if (currentDemo->reset()) {
	    currentDemo->exitPhysics();
	    currentDemo->initPhysics();
	}
    }

    return 0;
}
