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
static bool wait = true;

int Decide(int red_robots, int blue_robots, Uni::Robot& r)
{
	float red_self_preference = r.preferences[0]/r.preferences[1];
	float blue_self_preference = r.preferences[0]/r.preferences[2];
	if(1/(1 + (red_self_preference * (pow(eta,-(red_robots))))) > r.preferences[0])
		return 1;
	if(1/(1 + (blue_self_preference * (pow(eta,-(blue_robots))))) >  r.preferences[0])
		return 2;
	return 0;
}

void SpaceOut(Uni::Robot& r, Uni::Robot* other)
{
	double halfworld = Uni::worldsize * 0.5;
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

	r.dist_error[1] = r.dist_error[0];
	r.dist_error[0] = hypot(dx, dy);

	double proportional = r.dist_error[0] - 0.08;
	r.dist_integral = r.dist_integral + (proportional*Uni::sleep_msec);
	r.speed[0] = r.speed[0] + 0.004 * proportional;// + 0.0000004 * r.dist_integral;

	//printf("Speed for robot %d is %f\n", r.robot_number, r.speed[0] );

	if(fabs(r.dist_error[0] - 0.08) < 0.004) r.speed[0] = 0.005;
	if(r.speed[0] > 0.006) r.speed[0] = 0.006;
	if(r.speed[0] < 0.004) r.speed[0] = 0.004;

	return;
}

void FollowRobot(Uni::Robot& r, Uni::Robot* other){
	  double halfworld = Uni::worldsize * 0.5;
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

	  double absolute_heading = atan2( dy, dx );
	  double my_orientation = r.pose[2];
	  double relative_heading = fabs( other->pose[2] - my_orientation );

	  r.theta_error[1] = r.theta_error[0];
	  r.theta_error[0]= Uni::AngleNormalize((absolute_heading - my_orientation ));

	  double proportional = r.theta_error[0];
	  //double differential = (r.theta_error[0] - r.theta_error[1])/Uni::sleep_msec;
	  r.integral = r.integral + (r.theta_error[0]*Uni::sleep_msec);

	  r.speed[1] = 0.4 * proportional  + 0.0002 * r.integral;

	  if( relative_heading < 0.017) {
			  r.time_count++;
			  if (r.time_count > 5){

				  r.reward = true;

				  r.time_count = 0;
			  }
	  }

	  if(relative_heading > 0.017 || fabs(r.theta_error[0]) > 0.00017){
		  r.time_count = 0;
		  r.reward = false;
	  }
}

void FollowPoint(Uni::Robot& r, double x, double y){
	  double halfworld = Uni::worldsize * 0.5;
	  double dx = x - r.pose[0];

	  // wrap around torus
	  if( dx > halfworld )
		  dx -= Uni::worldsize;
	  else if( dx < -halfworld )
		  dx += Uni::worldsize;

	  double dy = y - r.pose[1];

	  // wrap around torus
	  if( dy > halfworld )
		  dy -= Uni::worldsize;
	  else if( dy < -halfworld )
		  dy += Uni::worldsize;

	  double absolute_heading = atan2( dy, dx );
	  double my_orientation = r.pose[2];

	  r.theta_error[1] = r.theta_error[0];
	  r.theta_error[0]= Uni::AngleNormalize((absolute_heading - my_orientation ));

	  double proportional = r.theta_error[0];
	  //double differential = (r.theta_error[0] - r.theta_error[1])/Uni::sleep_msec;
	  r.integral = r.integral + (r.theta_error[0]*Uni::sleep_msec);

	  r.speed[1] = 0.4 * proportional  + 0.0002 * r.integral;

	  if( fabs(r.pose[2]) < 0.017) {
			  r.time_count++;
			  if (r.time_count > 200){

				  r.reward = true;

				  r.time_count = 0;
			  }
	  }
}

// Examine the robot's pixels vector and set the speed sensibly.
void Controller( Uni::Robot& r, void* dummy_data )
{ 
  // steer away from the closest robot
  int closest = -1;
  //int closest_rewarded = -1;
  int closest_red = -1;
  int closest_blue = -1;

  double dist = r.range; // max sensor range
  int red_robots_inrange = 0;
  int blue_robots_inrange = 0;

  const size_t pixel_count = r.pixels.size();

  dist = r.range;

  for( unsigned int p=495; p<=505; p++ ){
	  unsigned int q = p;
	  //printf("%d\n", q);
  	  if( r.pixels[q].range < dist )
        {
  			  closest = (int)q;
  			  dist = r.pixels[q].range;
        }
    }

//  for( unsigned int p=250; p<=3*(pixel_count/4); p++ ){
//	  unsigned int q = p;
//  	  if( r.pixels[q].range < dist && r.pixels[q].robot->reward == true )
//        {
//  			  closest_rewarded = (int)q;
//  			  dist = r.pixels[q].range;
//        }
//    }

  dist = r.range;
   for( unsigned int p=250; p<=3*(pixel_count/4); p++ ){
	  unsigned int q = p;
 	  if( r.pixels[q].range < dist && r.pixels[q].robot->color[0] == 255)
       {
 			  closest_red = (int)q;
 			  dist = r.pixels[q].range;
       }
   }

  dist = r.range;
  for (unsigned int p=250; p<=3*(pixel_count/4); p++){
	  unsigned int q = p;
	  if( r.pixels[q].range < dist && r.pixels[q].robot->color[2] == 255)
	  {
			  closest_blue = (int)q;
			  dist = r.pixels[q].range;
	  }
  }

  for (unsigned int p=0; p<pixel_count; p++){
	  red_robots_inrange += r.pixels[p].other_robots[0];
	  blue_robots_inrange += r.pixels[p].other_robots[1];
  }

  if(closest < 0){
 	  r.speed[0] = r.speed_max;
   }

  if (r.change_lane == true){
 	if(r.reward == false){
  		  r.lane[0] = Uni::DistanceNormalize(r.lane[0] + r.speed[0]);
   		  FollowPoint(r, r.lane[0], r.lane[1]);
   		  printf("Follow Point\n");
   		  return;
 	}
   	else
   		  r.change_lane = false;
   }

   if(r.reward ==  true && r.change_lane == false){
 	  if(r.speed[0] != r.speed_max){
 		  r.change_lane = true;
 		  r.lane[0] = r.pose[0] + 0.04,
 		  r.lane[1] = r.pose[1] - 0.06;
 		  r.reward = false;
 		  return;
 	  }
   }

   if( closest_red < 0 && closest_blue < 0){ // nothing nearby: cruise
 	  return;
   }
//  if(closest_rewarded > -1){
//	  if ((r.pixels[closest_rewarded].robot->color[0] != r.color[0] ||
//			  r.pixels[closest_rewarded].robot->color[2] != r.color[2]) && r.reward == false )
//		  FollowRobot(r, r.pixels[closest_rewarded].robot );
//  }

//  A robot is nearby, decide whether to follow the closest robot
//  if(r.reward == true){
//	  if(r.color[0] == 255) SpaceOut(r, r.pixels[closest_red].robot);
//	  else SpaceOut(r, r.pixels[closest_blue].robot);
//  }

  if(closest > 0){
  	  if(r.speed_max > r.pixels[closest].robot->speed[0]){
		  r.speed[0] = r.pixels[closest].robot->speed[0];
  	  }
  	  else
  		  r.speed[0] = r.speed_max;
  }

//  if(wait){
//	  r.time_count++;
//	  if (r.time_count < 50)
//		  return;
//	  else{
//		  r.time_count = 0;
//		  wait = false;
//	  }
//  }

  int decision = Decide(red_robots_inrange, blue_robots_inrange, r);
  switch(decision)
  {
  	  case 0: break;
  	  case 1: if(closest_red > -1){
  		  	  	  Uni::Robot* other = r.pixels[closest_red].robot;
  		  	  	  FollowRobot(r, other);
  	  	  	  }
  	  	  	  break;
  	  case 2: if(closest_blue > -1){
  		  	  	  Uni::Robot* other = r.pixels[closest_blue].robot;
  		  	  	  FollowRobot(r, other);
  	  	  	  }
  	  	  	  break;
  }
}

inline void HighwayPose( double pose[3], double masterpose[3], int lanechoice)
{
    pose[0] = masterpose[0] = masterpose[0] - 0.04;

    if(lanechoice == 1)
  	  pose[1] = masterpose[1] - 0.08;
    else
  	  pose[1] = masterpose[1];

    pose[2] = masterpose[2];
}

int main( int argc, char* argv[] )
{
	srand(4);
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
	  int randomized_lane = rand() % 2 + 1;
	  count++;

//      if(count < 10){
//    	  Uni::RandomPose( r->pose );
//      	  r->robot_number = count;
//      }
      {
    	  HighwayPose( r->pose, masterpose, randomized_lane );
    	  r->robot_number = count;
      }

      // install our callback function
      r->callback = Controller;
      r->callback_data = NULL;
      if(r->color[0] == 255)
    	  r->speed[0] = 0.005;   // constant forward speed
      else
    	  r->speed[0] = 0.006;
      r->speed[1] = 0.0;     // no turning. we may change this below
    }
  
  // and start the simulation running
  Uni::Run();
  
  // we'll probably never get here, but this keeps the compiler happy.
  return 0;
}
