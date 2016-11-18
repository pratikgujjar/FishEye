/*****
		controller.cc
		version 3
		Copyright Richard Vaughan, 2013.1.10
****/

#include "universe.h"
#include <stdint.h>
#include <getopt.h>
#include <iostream>

static bool invert = false;

int Decide(int red_robots, int blue_robots, Uni::Robot r)
{
	float red_self_preference = r.preferences[0]/r.preferences[1];
	float blue_self_preference = r.preferences[0]/r.preferences[2];
	if(1/(1 + (red_self_preference * (pow(eta,-(red_robots))))) >  r.preferences[0])
		return 1;
	if(1/(1 + (blue_self_preference * (pow(eta,-(blue_robots))))) >  r.preferences[0])
		return 2;
	return 0;
}

// Examine the robot's pixels vector and set the speed sensibly.
void Controller( Uni::Robot& r, void* dummy_data )
{ 
  r.speed[0] = 0.005;   // constant forward speed 
  r.speed[1] = 0.0;     // no turning. we may change this below
  
  // steer away from the closest robot
  int closest = -1;
  int closest_red = -1;
  int closest_blue = -1;
  double dist = r.range; // max sensor range
  int red_robots_inrange = 0;
  int blue_robots_inrange = 0;
  
  double relative_pose = 0;
  double x_distance;
  double x_speed;
  unsigned int time;

  Uni::Robot bot_in_memory = r;

  const size_t pixel_count = r.pixels.size();

  for( unsigned int p=0; p<pixel_count; p++ ){
 	  if( r.pixels[p].range < dist)
       {
 			  closest = (int)p;
 			  dist = r.pixels[p].range;
       }
   }

  dist = r.range;
  for( unsigned int p=0; p<pixel_count; p++ ){
	  if( r.pixels[p].range < dist && r.pixels[p].robot->color[0] == 255 && r.pixels[p].robot->reward == true )
      {
			  closest_red = (int)p;
			  dist = r.pixels[p].range;
      }
  }

  dist = r.range;
  for (unsigned int p=0; p<pixel_count; p++){
	  if( r.pixels[p].range < dist && r.pixels[p].robot->color[2] == 255 && r.pixels[p].robot->reward == true )
	  {
			  closest_blue = (int)p;
			  dist = r.pixels[p].range;
	  }
	  red_robots_inrange += r.pixels[p].red_robots;
	  blue_robots_inrange += r.pixels[p].blue_robots;
  }

  if( closest_red < 0 && closest_blue < 0) // nothing nearby: cruise
    return;

  //A robot is nearby, decide whether to follow the closest robot
  int decision = Decide(red_robots_inrange, blue_robots_inrange, r);

  switch(decision)
  {
  	  case 0: /*if(closest > -1){
  		  	  	  if( closest < (int)pixel_count / 2 )
  		  	  		  r.speed[1] = 0.04; // rotate right
  		  	  	  else
  		  	  		  r.speed[1] = -0.04; // rotate left
  	  	  	  }*/
  	  	  	  break;
  	  case 1: if(closest_red > -1){
  		  	  	  //r.pose[2] = r.pixels[closest_red].robot->pose[2];
  		  	  	  if(r.pixels[closest_red].robot->pose[2] != r.pose[2]){
  		  	  		  printf("run once");
					  relative_pose = abs(r.pixels[closest_red].robot->pose[2] - r.pose[2]);
					  x_distance = abs(r.pixels[closest_red].robot->pose[1] - r.pose[1]);
					  time = Uni::sleep_msec;
					  x_speed = x_distance/time;
					  r.speed[0] = x_speed/sin(relative_pose);
					  r.speed[1] = (r.pixels[closest_red].robot->pose[2] - r.pose[2]); // rotate right
					  r.reward = true;
  		  	  	  }
  	  	  	  }
  	  	  	  break;
  	  case 2: if(closest_blue > -1)
  		  	  	  //r.pose[2] = r.pixels[closest_blue].robot->pose[2];
  		  	  	  if(r.pixels[closest_blue].robot->pose[2] != r.pose[2])
  		  		  	  r.speed[0] = 2 * 0.005;
  		  	  	  r.speed[1] = (r.pixels[closest_blue].robot->pose[2] - r.pose[2]); // rotate right
  	  	  	  	  r.reward = true;
  	  	  	  break;

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
    	std::cout << "Inverting enabled" << '\n';
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
