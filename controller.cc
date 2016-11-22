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

int Decide(int red_robots, int blue_robots, Uni::Robot r)
{
	float red_self_preference = r.preferences[0]/r.preferences[1];
	float blue_self_preference = r.preferences[0]/r.preferences[2];
	if(1/(1 + (red_self_preference * (pow(eta,-(red_robots))))) >  0)
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
  int farthest_red = -1;
  double dist = r.range; // max sensor range
  int red_robots_inrange = 0;
  int blue_robots_inrange = 0;
  static int number_manuevers = 0;
  
  double halfworld = Uni::worldsize * 0.5;
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
  double dist2 = 0;
  for( unsigned int p=0; p<pixel_count; p++ ){
	  if( r.pixels[p].range < dist && r.pixels[p].robot->color[0] == 255 && r.pixels[p].robot->reward == true )
      {
			  closest_red = (int)p;
			  dist = r.pixels[p].range;
      }
	  if( r.pixels[p].range != r.range && r.pixels[p].range > dist2 && r.pixels[p].robot->color[0] == 255 && r.pixels[p].robot->reward == true )
	  {
			  farthest_red = (int)p;
			  dist2 = r.pixels[p].range;
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
  		  	  	  Uni::Robot* other = &(*r.pixels[closest_red].robot);

  		  	  	  double dx = other->pose[0] - r.pose[0];

  		  	  	  // wrap around torus
				  if( dx > halfworld )
					  dx -= Uni::worldsize;
				  else if( dx < -halfworld )
					  dx += Uni::worldsize;

				  double dy = other->pose[1] - r.pose[1];

				  // wrap around torus
				  if( dy > halfworld )
					  dy -= Uni::worldsize;
				  else if( dy < -halfworld )
					  dy += Uni::worldsize;

				  double range = hypot( dx, dy );

  		  	  	  double absolute_heading = atan2( dy, dx );
  		  	  	  double my_heading = r.pose[2];
				  //double relative_heading = Uni::AngleNormalize((absolute_heading - r.pose[2]) );
				  //double relative_orientation = Uni::AngleNormalize(other->pose[2] - r.pose[2] ); //negative means right, positive means other is on the left

				  double theta_error = absolute_heading - my_heading;
				  r.speed[1] = 0.2 * theta_error;

				  //r.reward = true;
  	  	  	  }
  	  	  	  break;
  	  case 2: if(closest_blue > -1)
  		  	  	  //r.pose[2] = r.pixels[closest_blue].robot->pose[2];
  		  	  	  r.pose[2] = (r.pixels[closest_blue].robot->pose[2]); // rotate right
  	  	  	  	  r.reward = true;
  	  	  	  break;

  }
}

int main( int argc, char* argv[] )
{
  // configure global robot settings
  Uni::Init( argc, argv );
  double masterpose[3];
  int count = 0;
  
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
  masterpose[0] = drand48() * Uni::worldsize;
  masterpose[1] = drand48() * Uni::worldsize;
  masterpose[2] = Uni::AngleNormalize( Uni::dtor(0));
  
  // configure the robots the way I want 'em
  FOR_EACH( r, Uni::population )
    {
	  //count++;
      //if(count < 20)
    	  Uni::RandomPose( r->pose );
      /*else{
    	  Uni::HighwayPose( r->pose, masterpose, r->color );
    	  r->reward = true;
      }*/

      // install our callback function
      r->callback = Controller;
      r->callback_data = NULL;
    }
  
  // and start the simulation running
  Uni::Run();
  
  // we'll probably never get here, but this keeps the compiler happy.
  return 0;
}
