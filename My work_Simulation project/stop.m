% Stop the robot
velmsg.linear.x = 0;
velmsg.angular.z = 0;
send(PubVel,velmsg);
