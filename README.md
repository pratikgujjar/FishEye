FishEye
========

An emulation of the Fish Schooling principle to n distributed robots.
Simulator and base forked from rtv/universe.

Different laning implementations are maintained in separate branches.
Branch: BotsJoiningGroups 
Description: Associating rewards with robots that decide to form a group. Leads to separate flocking of red and blue robots
git checkout origin experimental-BotsJoiningGroups

Branch: AutoDriving
Description: Schooling robots form lanes to achieve maximum system speed. Two lanes of traffic
git checkout origin experimental-AutoDriving

Branch: ThreeLaning
Description: Schooling robots separate to form three lanes of traffic
git checkout origin experimental-ThreeLaning

For Mac sytems, change #include<Gl/glut.h> to #include<glut/glut.h> in universe.cc file.

To run:
make
./universe

Author: Pratik Gujjar 2016.
License: GNU GPL v3 or later (applies to all files in this repo).

