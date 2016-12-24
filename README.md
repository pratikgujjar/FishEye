FishEye
========

An emulation of the Fish Schooling principle to n distributed robots. <br />
Simulator and base forked from rtv/universe. <br />

Different laning implementations are maintained in separate branches. <br />
Branch: BotsJoiningGroups <br />
Description: Associating rewards with robots that decide to form a group. Leads to separate flocking of red and blue robots <br />
$ git checkout origin experimental-BotsJoiningGroups <br />

Branch: AutoDriving <br />
Description: Schooling robots form lanes to achieve maximum system speed. Two lanes of traffic <br />
$ git checkout origin experimental-AutoDriving <br />

Branch: ThreeLaning <br />
Description: Schooling robots separate to form three lanes of traffic <br />
$ git checkout origin experimental-ThreeLaning <br />

For Mac sytems, change #include< Gl/glut.h > to #include< glut/glut.h > in universe.cc file. <br />

To run: <br />
$ make <br />
$ ./universe <br />

Author: Pratik Gujjar 2016. <br />
License: GNU GPL v3 or later (applies to all files in this repo).

