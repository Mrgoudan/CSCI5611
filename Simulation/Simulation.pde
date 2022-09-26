
float agentSpeed = 100;

//The agent states
Vec2 agentPos;
Vec2 agentVel;
Vec2 point1;
Vec2 point2;
float agentRad = 30;

// Vec2 agentAcc;

//The agent goals
//Vec2 goalPos;

ArrayList<Vec2> path;

int len;
int i;



void setup(){
  placeRandomObstacles(numObstacles);
  size(850,650);
  //size(850,650,P3D); //Smoother
  surface.setTitle("2D simulation");
  //Set initial agent positions and goals
  path = new ArrayList<>():
  agentVel = new Vec2(0.0,0.0);
  agentPos= new Vec2(400,610);
  
  goalPos = new Vec2(200,420);
  point1 = new Vec2(300,300);
  point2 = new Vec2(250,300);
  
  path.add(point1);
  path.add(point2);
  path.add(goalPos);


  len = path.size();
  i = 0;
  //Set initial velocities to cary agents towards their goals
    // agentVel[i] = goalPos[i].minus(agentPos[i]);
    // agentVel[i].setToLength(goalSpeed);

}




//Update agent positions & velocities based acceleration
void moveAgent(float dt){
  //Compute accelerations for every agents
  Vec2 goal = path.get(i);
  Vec2 dir = goal.minus(agentPos);
  if(dir.length() < agentSpeed*dt){
    agentVel = new Vec2(0,0);
    agentPos = goal;
    if(i < len - 1) i++;
  }else{
    dir.normalize();
    agentVel = dir.times(agentSpeed*dt);
  }

  agentPos.add(agentVel);

  // for (int i = 0; i < numAgents; i++){
  //   agentAcc[i] = computeAgentForces(i);
  // }
  // //Update position and velocity using (Eulerian) numerical integration
  // for (int i = 0; i < numAgents; i++){
  //   agentVel[i].add(agentAcc[i].times(dt));
  //   agentPos[i].add(agentVel[i].times(dt));
  // }
}

boolean paused = true;
void draw(){
  background(255,255,255); //White background
  stroke(0,0,0);
  fill(255,255,255);
   for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    circle(c.x,c.y,r*2);
  }

  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
 
  //Draw orange goal rectangle
   fill(255,150,50);
   rect(goalPos.x, goalPos.y, 20, 20);

 
  //Draw the green agents
  fill(20,200,150);

  circle(agentPos.x, agentPos.y, (agentRad - 10)*2);  //No.4 smaller radius

}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') paused = !paused;
}


///////////////////////

// float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 
//   //Compute displacement vector pointing from the start of the line segment to the center of the circle
//   Vec2 toCircle = center.minus(l_start);
 
//   //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
//   float a = l_dir.length()*l_dir.length();
//   float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
//   float c = toCircle.lengthSqr() - (r*r); //different of squared distances
 
//   float d = b*b - 4*a*c; //discriminant
 
//   if (d >=0 ){
//     //If d is positive we know the line is colliding
//     float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
//     if (t >= 0) return t;
//     return -1;
//   }
 
//   return -1; //We are not colliding, so there is no good t to return
// }
