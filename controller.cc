/*****
		controller.cc
		version 3
		Copyright Richard Vaughan, 2013.1.10
****/

#include "universe.h"
#include <stdint.h>
#include <getopt.h>
#include <iostream>
static bool invert = true;

bool Decide(int num, Uni::Robot r)
{
	float self_information = r.preferences[0]/r.preferences[1];
	if(1/(1 + (self_information * (pow(eta,-(num))))) >  r.preferences[0])
		return true;
	else
		return false;
}

// Examine the robot's pixels vector and set the speed sensibly.
void Controller( Uni::Robot& r, void* dummy_data )
{ 
  r.speed[0] = 0.005;   // constant forward speed 
  r.speed[1] = 0.0;     // no turning. we may change this below
  
  // steer away from the closest robot
  int closest = -1;
  double dist = r.range; // max sensor range
  int happy_robots_count = 0;
  
  const size_t pixel_count = r.pixels.size();
  for( unsigned int p=0; p<pixel_count; p++ )
    if( r.pixels[p].range < dist )
      {
    	closest = (int)p;
    	dist = r.pixels[p].range;
    	happy_robots_count += r.pixels[p].happy_robots_count;
      }
  
  if( closest < 0 ) // nothing nearby: cruise
    return;

  //A robot is nearby, decide whether to follow the closest robot
  if (Decide(happy_robots_count, r) == true)
  {
	//std::cout << "Decision True" << '\n' << happy_robots_count << '\n';
	//std::cout << r.pose[2] << '\n';
	//std::cout << r.pixels[closest].robot->pose[2] << '\n';
	r.pose[2] = r.pixels[closest].robot->pose[2];
  }
}

int main( int argc, char* argv[] )
{
  // configure global robot settings
  Uni::Init( argc, argv );
  
  // parse remaining cmdline arguments to configure swarm
  int c=0;
  while( ( c = getopt( argc, argv, "i")) != -1 )
    switch( c )
      {
      case 'i': invert = true;
	puts( "[Ctrl] invert" );
	break;				
      }
  
  // configure the robots the way I want 'em
  FOR_EACH( r, Uni::population )
    {
      Uni::RandomPose( r->pose ); 
      
      // install our callback function
      r->callback = Controller;
      r->callback_data = NULL;
    }
  
  // and start the simulation running
  Uni::Run();
  
  // we'll probably never get here, but this keeps the compiler happy.
  return 0;
}
