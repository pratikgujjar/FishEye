/****
     universe.h
     Clone this package from git://github.com/rtv/universe.git
     version 2
     Richard Vaughan  
****/

#include <vector>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#define GRAPHICS 1
#define PI 3.1415

// handy STL iterator macro pair. Use FOR_EACH(I,C){ } to get an iterator I to
// each item in a collection C.
#define VAR(V,init) __typeof(init) V=(init)
#define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)

// define the social information parameter
#define eta 1.55

namespace Uni
{
  class Robot;
  
  /** initialization: call this before using any other calls. */	
  void Init( int argc, char** argv );

  /** update all robots */
  void UpdateAll();
  
  
  /** Start running the simulation. Does not return. */
  void Run();
  
  extern uint64_t updates; // number of simulation steps so far	 
  extern uint64_t updates_max; // number of steps to run before quitting (0 means infinity)
  extern double worldsize; // side length of the toroidal world
  extern unsigned int sleep_msec;
  
  class Robot
  {
  public:
    // static data members (same for all instances)
    static double range;      // sensor detects objects up to this maximum distance
    static double fov;        // sensor detects objects within this angular field-of-view     
    static unsigned int  pixel_count; // number of pixels in sensor array
    
    // non-static data members
    double pose[3] ;   // 2d pose and orientation [0]=x, [1]=y, [2]=a;
    double speed[2];   // linear speed [0] and angular speed [1]
    uint8_t color[3];  // body color [0]=red, [1]=green, [2]=blue
    bool reward; 	   // Is the robot rewarded
    int time_count;
    double theta_error[2];
    double integral;
    double dist_error[2];
    double dist_integral;
    // Addition to support Bayesian decisions
    float preferences[3]; // choice[0] = probability of laziness, choice[1] = probability of joining red group, choice[2] = joining blue group

    class Pixel 
    {
    public:
      double range; // between zero and Robot::range
      Robot* robot; // closest robot detected or NULL if nothing detected
      int red_robots; // number of rewarded red robots in the pixel
      int blue_robots;  // number of rewarded blue robots in the pixel
    };
    
    std::vector<Pixel> pixels; // sensor array
    
    // default constructor
    Robot( void );
    
    // default destructor
     ~Robot() {}
        
    // render the robot in OpenGL
    void Draw() const;
    
    // move the robot
    void UpdatePose();
    
    // update
    void UpdateSensor();

    // Determine Boss
    Robot DetermineBoss();

    // callback function for controlling this robot
    void (*callback)( Robot& r, void* user );
    void* callback_data;
  };	
  
  extern std::vector<Robot> population;
  
  // utilities
  
  /** Normalize a length to within 0 to worldsize. */
  inline double DistanceNormalize( double d )
  {
    while( d < 0 ) d += worldsize;
    while( d > worldsize ) d -= worldsize;
    return d; 
  } 
  
  /** Normalize an angle to within +/_ M_PI. */
  inline double AngleNormalize( double a )
  {
    while( a < -M_PI ) a += 2.0*M_PI;
    while( a >  M_PI ) a -= 2.0*M_PI;	 
    return a;
  }	 
  
  /** Convert radians to degrees. */
  inline double rtod( double r )
  { return( r * 180.0 / M_PI ); }
  
  /** Convert degrees to radians */
  inline double dtor( double d)
  { return( d * M_PI / 180.0 ); }
  
  inline void RandomPose( double pose[3] )
  {
    pose[0] = drand48() * worldsize;
    pose[1] = drand48() * worldsize;
    pose[2] = AngleNormalize( drand48() * (M_PI*2.0));
  }

  inline void HighwayPose( double pose[3], double masterpose[3], uint8_t colour[3])
    {
      pose[0] = masterpose[0] = masterpose[0] - 0.04;

      if(colour[0] == 255)
    	  pose[1] = masterpose[1] - 0.24;
      else
    	  pose[1] = masterpose[1];
      pose[2] = masterpose[2];
    }
  
}; // namespace Uni
