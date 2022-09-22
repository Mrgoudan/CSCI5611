//CSCI 5611 - Pathing and Crowd Sim
//Anticipatory Collision Avoidance (TTC-Forces) [Exercise]
// Stephen J. Guy <sjguy@umn.edu>

//NOTE: The simulation starts paused! Press the spacebar to run it.

/*
TODO:
  1. Currently, there are no forces acting on the agents. Add a goal force that penalizes
     the difference between an agent's current velocity and it's goal velocity.
     i.e., goal_force = goal_vel-current_vel; acc += goal_force
  1a. Scale the goal force by the value k_goal. Find a value of k_goal that lets the agents
     smoothly reach their goal without overshooting it first.
     i.e., goal_force = k_goal*(goal_vel-current_vel); acc += goal_force
  2. Finish the function computeTTC() so that it computes the time-to-collision (TTC) between
     two agents given their positions, velocities and radii. Your implementation should likely
     call rayCircleIntersectTime to compute the time a ray intersects a circle as part of
     determining the time to collision. (i.e., at what time would a ray moving at your
     current relative velocity intersect the Minkowski sum of the obstacle and your reflection?)
     If there is no collision, return a TTC of -1.
     ie., TTC(A,B) = Ray-Disk(A's position, relative velocity, B's position, combined radius)
  2a. Test computeTTC() with some scenarios where you know the (approximate) true value.
  3. Compute an avoidance force as follows:
       - Find the ttc between agent "id" and it's neighbor "j" (make sure id != j)
       - If the ttc is negative (or there is no collision) then there is no avoidance force w.r.t agent j
       - Predict where both agent's well be at the moment of collision (time = TTC)
         ie., A_future = A_current + A_vel*ttc; B_future + B_current + B_vel*ttc
       - Find the relative vector between the agents at the moment of collision, normalize it
         i.e, relative_future_direction = (A_future - B_future).normalized()
       - Compute the per-agent avoidance force by scaling this direction by k_avoid and
         dividing by ttc (smaller ttc -> more urgent collisions)
         acc += k_avoid * (1/ttc) * relative_future_direction
      The final force should be the avoidance force for each neighbor plus the overall goal force.
      Try to find a balance between k_avoid and k_goal that gives good behavior. Note: you may need a
      very large value of k_goal to avoid collisions between agents.
   4. Make the following simulation improvements:
     - Draw agents with a slightly smaller radius than the one used for collisions
     - When an agent reaches its goal, stop it from reacting to other agents
   5. Finally, place 2 more agents in the scene and try to make an interesting scenario where the
      agents interact with each other.

CHALLENGE:
  1. Give agents a maximum force and maximum velocity and don't let them violate these constraints.
  2. Start two agents slightly colliding. Update the approach to better handle collisions.
  3. Add a static obstacle for the agent to avoid (hint: treat it as an agent with 0 velocity).
*/

static int maxNumAgents = 5;
int numAgents = 5;
int numObs =10;
float k_goal = 5;  //TODO: Tune this parameter to agent stop naturally on their goals
float k_avoid = 200;
float agentRad = 25;
float goalSpeed = 150;

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

//The agent goals
Vec2[] goalPos = new Vec2[maxNumAgents];

//obs
Vec2[] obsPos = new Vec2[numObs];

void setup(){
  size(850,650);
  resetSimulation();
  // size(850,650,P3D); //Smoother


}

void resetSimulation(){
  //Set initial agent positions and goals
  for (int i = 0; i < numAgents; i++){ 
    agentPos[i] = new Vec2(random(850),random(650)); 
    agentPos[i] = new Vec2(random(850),random(650)); 
    goalPos[i] = new Vec2(random(850),random(650));

  }
  for(int i = 0; i < numObs; i++){
    obsPos[i] = new Vec2(random(850),random(650));
  }
  
  //Set initial velocities to cary agents towards their goals
  for (int i = 0; i < numAgents; i++){
    agentVel[i] = goalPos[i].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed);
  }
}

//When will agents 1 and 2 collide if they keep their current velocities?
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  //return -1;
  float combinedRadius = radius1+radius2;
  Vec2 relativeVelocity = vel1.minus(vel2);
  float ttc = rayCircleIntersectTime(pos2, combinedRadius, pos1, relativeVelocity);
  if (ttc==0) println(ttc);
  return ttc;
}

// Compute attractive forces to draw agents to their goals, 
// and avoidance forces to anticipatory avoid collisions
Vec2 computeAgentForces(int id){
  //TODO: Make this better
  Vec2 acc = new Vec2(0,0);
  
  
  Vec2 goalVel = goalPos[id].minus(agentPos[id]);
  if (goalVel.length() > goalSpeed) goalVel.setToLength(goalSpeed);
  Vec2 goalForce = (goalVel.minus(agentVel[id]));
  acc.add(goalForce.times(k_goal));
  
  if (goalVel.length() < 5) return acc;
  

  for (int j = 0; j < numAgents; j++){
    if (j == id) continue;
    float ttc = computeTTC(agentPos[id],agentVel[id], agentRad, agentPos[j],agentVel[j], agentRad);
    
    Vec2 futurePos_id = agentPos[id].plus(agentVel[id].times((ttc+0.00001)));
    Vec2 futurePos_j = agentPos[j].plus(agentVel[j].times((ttc+0.00001)));
    Vec2 avoidDir = futurePos_id.minus(futurePos_j).normalized();
    Vec2 avoidForce = avoidDir.times(1/(ttc+0.0001));
    //println(ttc, avoidForce.x, avoidForce.y);
    if (ttc > 0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
    if (ttc==0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
  }
  for (int j = 0; j < numObs; j++){
    float ttc = computeTTC(agentPos[id],agentVel[id], agentRad, obsPos[j],new Vec2(0,0), 10*sqrt(2));
    
    Vec2 futurePos_id = agentPos[id].plus(agentVel[id].times((ttc+0.00001)));
    Vec2 futurePos_j = obsPos[j].plus(new Vec2(0,0));
    Vec2 avoidDir = futurePos_id.minus(futurePos_j).normalized();
    Vec2 avoidForce = avoidDir.times(1/(ttc+0.0001));
    //println(ttc, avoidForce.x, avoidForce.y);
    if (ttc > 0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
    if (ttc==0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
  }
  
  return acc;
}


//
void moveAgent(float dt){
  //Compute accelerations for every agents
  for (int i = 0; i < numAgents; i++){
    agentAcc[i] = computeAgentForces(i);
  }
  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < numAgents; i++){
    agentVel[i].add(agentAcc[i].times(dt));
    agentPos[i].add(agentVel[i].times(dt));
  }
}

boolean paused = true;
void draw(){
  background(255,255,255); //White background
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
  
  //Draw goal rectangle
  fill(255,50,50);
  for (int i = 0; i < numAgents; i++){
    rect(goalPos[i].x-5, goalPos[i].y-5, 10, 10);
  }
  
  //Draw the agents
  fill(20,120,175);
  for (int i = 0; i < numAgents; i++){
    circle(agentPos[i].x, agentPos[i].y, agentRad*1.95); //Shrink drawn radius
  }
  fill(123,123,123);
  for (int i = 0; i < numObs; i++){
    rect(obsPos[i].x-10, obsPos[i].y-10, 20,20); //Shrink drawn radius
  }
}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') paused = !paused;
  if (key == 'r') resetSimulation();
}

///////////////////////
float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
  
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length()*l_dir.length(); 
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
  
  float d = b*b - 4*a*c; //discriminant 
  
  if (d >=0 ){ 
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision! 
    // println(t);
    if(((-b - sqrt(d))/(2*a)<0 && (-b + sqrt(d))/(2*a)>0)||((-b - sqrt(d))/(2*a)>0 && (-b + sqrt(d))/(2*a)<0)){
      return 0;
    }
    if (t >= 0) return t;
    return -1;
  }
  
  return -1; //We are not colliding, so there is no good t to return 
}

//////////////////////
//Vector Library
//CSCI 5611 Vector 2 Library [Example]
//////////////////////

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public float lengthSqr(){
    return x*x+y*y;
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}
