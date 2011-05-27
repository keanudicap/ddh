/*
 *  RunTests.cpp
 *  hog
 * 
 *	main() code borrowed from http://www.evocomp.de/tutorials/tutorium_cppunit/howto_tutorial_cppunit_en.html
 *
 *  Created by Daniel Harabor on 29/11/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 */

#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/TestResult.h>
#include <cppunit/TestResultCollector.h>
#include <cppunit/TestRunner.h>
#include <cppunit/BriefTestProgressListener.h>

#include "graph.h"
#include "HPAClusterAbstraction.h"
#include "HPAClusterAbstraction.h"
#include "TestConstants.h"

#include "HPAClusterFactory.h"
#include "ClusterNodeFactory.h"
#include "ClusterAStarFactory.h"
#include "EdgeFactory.h"
#include "NodeFactory.h"
#include "EdgeFactory.h"

#include <iostream>

int runtests(void);

int main (int argc, char* argv[])
{
	runtests();	
//	sleep(60);
	return 0;
}

int runtests()
{

    // informs test-listener about testresults
    CPPUNIT_NS :: TestResult testresult;

    // register listener for collecting the test-results
    CPPUNIT_NS :: TestResultCollector collectedresults;
    testresult.addListener (&collectedresults);

    // register listener for per-test progress output
    CPPUNIT_NS :: BriefTestProgressListener progress;
    //testresult.addListener (&progress);

    // insert test-suite at test-runner by registry
    CPPUNIT_NS :: TestRunner testrunner;
    testrunner.addTest (CPPUNIT_NS :: TestFactoryRegistry :: getRegistry ().makeTest ());
    testrunner.run (testresult);

    // output results in compiler-format
    CPPUNIT_NS :: CompilerOutputter compileroutputter (&collectedresults, std::cerr);
    compileroutputter.write ();

    // return 0 if tests were successful
    return collectedresults.wasSuccessful () ? 0 : 1;
}
